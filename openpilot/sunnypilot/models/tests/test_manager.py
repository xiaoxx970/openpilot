import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import MagicMock, patch

from openpilot.common.file_chunker import get_chunk_name, get_manifest_path
from openpilot.sunnypilot.models.manager import ModelManagerSP


class TestModelManager(TestCase):
  @staticmethod
  def _new_manager():
    manager = ModelManagerSP.__new__(ModelManagerSP)
    manager.params = MagicMock()
    manager.params.get.return_value = b"1"
    manager.selected_bundle = None
    manager._chunk_size = 4
    manager._download_start_times = {}
    manager._report_status = MagicMock()
    return manager

  @staticmethod
  def _response(data: bytes):
    response = MagicMock()
    response.headers = {"content-length": str(len(data))}
    response.iter_content.return_value = [data]
    response.__enter__.return_value = response
    return response

  def test_download_file_streams_with_device_http_dependency(self):
    manager = self._new_manager()

    response = self._response(b"12345678")
    response.iter_content.return_value = [b"1234", b"5678"]
    artifact = SimpleNamespace(fileName="model.onnx", downloadProgress=SimpleNamespace(status=None, progress=0.0, eta=0))
    with tempfile.TemporaryDirectory() as temp_dir:
      destination = Path(temp_dir) / artifact.fileName
      with patch("openpilot.sunnypilot.models.manager.requests.get", return_value=response) as get:
        asyncio.run(manager._download_file("https://example.com/model.onnx", str(destination), artifact))

      get.assert_called_once_with("https://example.com/model.onnx", stream=True, timeout=60)
      response.raise_for_status.assert_called_once_with()
      assert destination.read_bytes() == b"12345678"
    assert artifact.downloadProgress.progress == 100.0
    assert artifact.fileName not in manager._download_start_times

  def test_download_chunked_streams_each_chunk(self):
    manager = self._new_manager()
    responses = [self._response(b"1234"), self._response(b"5678")]
    artifact = SimpleNamespace(fileName="model.onnx", chunks=[object(), object()],
                               downloadProgress=SimpleNamespace(status=None, progress=0.0, eta=0))

    with tempfile.TemporaryDirectory() as temp_dir:
      destination = str(Path(temp_dir) / artifact.fileName)
      url = "https://example.com/model.onnx"
      with patch("openpilot.sunnypilot.models.manager.requests.get", side_effect=responses) as get:
        asyncio.run(manager._download_chunked(url, destination, artifact))

      assert Path(get_chunk_name(destination, 0, 2)).read_bytes() == b"1234"
      assert Path(get_chunk_name(destination, 1, 2)).read_bytes() == b"5678"
      assert Path(get_manifest_path(destination)).read_text() == "2"
      assert [call.args[0] for call in get.call_args_list] == [get_chunk_name(url, 0, 2), get_chunk_name(url, 1, 2)]

    assert artifact.downloadProgress.progress == 99.0
    assert artifact.fileName not in manager._download_start_times
