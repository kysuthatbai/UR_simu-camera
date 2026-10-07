"""LLM client toi 9Router (API tuong thich OpenAI: POST {base_url}/chat/completions).

Chi dung thu vien chuan (urllib) de khong can cai them package.
"""

import json
import urllib.error
import urllib.request
from typing import Dict, List


class LLMError(RuntimeError):
    pass


class LLMClient:
    def __init__(self, base_url: str, model: str, api_key: str = "", timeout: float = 60.0,
                 temperature: float = 0.0, max_tokens: int = 800):
        self.url = base_url.rstrip("/") + "/chat/completions" if base_url else ""
        self.model = model
        self.api_key = api_key
        self.timeout = timeout
        self.temperature = temperature
        self.max_tokens = max_tokens

    def chat(self, messages: List[Dict[str, str]]) -> str:
        if not self.url:
            raise LLMError("Chua cau hinh llm.base_url")
        if not self.model:
            raise LLMError("Chua cau hinh llm.model (ten model/combo trong 9Router), "
                           "vd: --ros-args -p llm.model:=<ten_model>")
        body = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            "stream": False,
        }
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        req = urllib.request.Request(
            self.url, data=json.dumps(body).encode("utf-8"), headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", errors="replace")[:500]
            raise LLMError(f"HTTP {e.code} tu {self.url}: {detail}") from e
        except urllib.error.URLError as e:
            raise LLMError(f"Khong ket noi duoc {self.url} ({e.reason}). "
                           "9Router da chay chua?") from e
        except (TimeoutError, json.JSONDecodeError) as e:
            raise LLMError(f"Loi khi goi LLM: {e}") from e

        try:
            return payload["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as e:
            raise LLMError(f"Phan hoi LLM khong dung dinh dang OpenAI: {str(payload)[:500]}") from e
