from pathlib import Path
import re
import shutil
import time
from urllib.parse import quote

import requests
from filelock import FileLock
from molmospaces_resources import R2RemoteStorage


class ResumableR2Storage(R2RemoteStorage):
    """Cached downloads with .part-file resume support."""

    def __init__(self, source: str, download_dir: Path):
        super().__init__(source)
        self.download_dir = download_dir

    def prepare_catalog(self, relative_path: Path) -> Path:
        from molmospaces_resources.constants import COMBINED_TRIES_NAME, REMOTE_MANIFEST_NAME

        for filename in (REMOTE_MANIFEST_NAME, COMBINED_TRIES_NAME):
            self._download(relative_path / filename)
        return self.download_dir / relative_path

    def fetch_file(self, relative_path, filename, dest):
        shutil.copyfile(self._download(relative_path / filename), dest / filename)

    def stream_archive(self, relative_path, archive_name):
        return self._download(relative_path / archive_name).open("rb")

    def _download(self, relative_path: Path) -> Path:
        dest = self.download_dir / relative_path
        dest.parent.mkdir(parents=True, exist_ok=True)
        with FileLock(str(dest) + ".lock"):
            if dest.is_file():
                return dest
            return self._transfer(relative_path, dest)

    def _transfer(self, relative_path: Path, dest: Path) -> Path:
        partial = dest.with_name(dest.name + ".part")
        url = f"{self.base_url.rstrip('/')}/{quote(relative_path.as_posix())}"
        failures = 0
        while True:
            offset = partial.stat().st_size if partial.exists() else 0
            headers = {"Accept-Encoding": "identity"}
            if offset:
                headers["Range"] = f"bytes={offset}-"
            try:
                with requests.get(url, headers=headers, stream=True, timeout=(15, 30)) as response:
                    if offset and response.status_code == 416:
                        if response.headers.get("Content-Range") == f"bytes */{offset}":
                            return partial.replace(dest)
                    response.raise_for_status()
                    if response.status_code == 206:
                        content_range = response.headers.get("Content-Range", "")
                        match = re.fullmatch(r"bytes (\d+)-(\d+)/(\d+)", content_range)
                        if not match or int(match[1]) != offset:
                            raise IOError(f"Unexpected Content-Range for {url}: {content_range}")
                        total = int(match[3])
                        mode = "ab"
                    else:
                        total = int(response.headers["Content-Length"])
                        mode = "wb"
                    with partial.open(mode) as output:
                        for chunk in response.iter_content(chunk_size=64 * 1024):
                            output.write(chunk)
                    size = partial.stat().st_size
                    if size == total:
                        return partial.replace(dest)
                    if size > total:
                        partial.unlink()
                        raise IOError(f"Received more than the expected {total} bytes for {url}")
            except requests.HTTPError as exc:
                if exc.response.status_code not in {408, 429, 500, 502, 503, 504}:
                    raise
            except (requests.ConnectionError, requests.Timeout, requests.exceptions.ChunkedEncodingError):
                pass

            size = partial.stat().st_size if partial.exists() else 0
            failures = 0 if size > offset else failures + 1
            if failures == 5:
                raise IOError(f"Download stalled at {size} bytes for {url}. Rerun to resume.")
            if failures:
                time.sleep(failures)
