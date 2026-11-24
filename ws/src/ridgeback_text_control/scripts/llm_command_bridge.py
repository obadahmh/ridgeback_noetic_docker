#!/usr/bin/env python3
"""Translate natural language into safe Ridgeback commands via K2-Think.

The LLM never publishes to ROS. It only emits JSON, which is then validated by
text_command_executor before optional execution.
"""

import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request
from typing import Any, Dict

import rospy

for candidate in (
    "/home/obadah/ws/src/ridgeback_text_control/scripts",
    os.path.dirname(os.path.abspath(__file__)),
):
    module_path = os.path.join(candidate, "text_command_executor.py")
    if os.path.exists(module_path) and candidate not in sys.path:
        sys.path.insert(0, candidate)

from text_command_executor import (
    CommandError,
    TextCommandExecutor,
    describe_command,
    parse_json_command,
    safety_from_args,
)


SYSTEM_PROMPT = """You translate human robot-control requests into one JSON object.

Output JSON only. No markdown. No explanations.

Allowed schemas:
{"action":"move","direction":"forward","distance_m":0.3}
{"action":"move","direction":"back","distance_m":0.2}
{"action":"turn","direction":"left","angle_deg":30}
{"action":"turn","direction":"right","angle_deg":45}
{"action":"stop"}
{"action":"status"}

Rules:
- Use only action, direction, distance_m, angle_deg, reason.
- Never include velocity, ROS topics, code, shell commands, or multiple commands.
- If the request is ambiguous, unsafe, too broad, or asks for autonomy/navigation, output {"action":"stop","reason":"ambiguous_or_unsafe"}.
- Maximum distance is 1.0 m. Maximum turn is 90 degrees.
- Convert common units into meters or degrees.
"""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="K2-Think natural-language bridge for bounded Ridgeback text commands."
    )
    parser.add_argument("request", nargs="*", help="Natural language command.")
    parser.add_argument("--dry-run", action="store_true", help="Validate only; do not start ROS or publish.")
    parser.add_argument("--base-url", default=os.environ.get("K2_THINK_BASE_URL") or os.environ.get("K2_THINK_API_BASE") or "http://localhost:8000/v1")
    parser.add_argument("--model", default=os.environ.get("K2_THINK_MODEL", "LLM360/K2-Think-V2"))
    parser.add_argument("--api-key-env", default="K2_THINK_API_KEY", help="Environment variable containing API key.")
    parser.add_argument("--llm-timeout", type=float, default=30.0, help="LLM HTTP request timeout, seconds.")
    parser.add_argument("--show-prompt", action="store_true", help="Print the system prompt and exit.")

    parser.add_argument("--cmd-topic", default="/cmd_vel", help="Velocity command topic.")
    parser.add_argument("--odom-topic", default="/odometry/filtered", help="Odometry topic used for bounds.")
    parser.add_argument("--max-distance", type=float, default=1.0, help="Max distance per command, meters.")
    parser.add_argument("--max-angle", type=float, default=90.0, help="Max angle per command, degrees.")
    parser.add_argument("--linear-speed", type=float, default=0.12, help="Linear speed, m/s.")
    parser.add_argument("--angular-speed", type=float, default=0.35, help="Angular speed, rad/s.")
    parser.add_argument("--timeout-motion", dest="timeout", type=float, default=12.0, help="Max seconds per motion command.")
    parser.add_argument("--odom-timeout", type=float, default=1.0, help="Fresh odom timeout, seconds.")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Publish/control loop rate.")
    return parser


def chat_completion(base_url: str, model: str, api_key: str, user_request: str, timeout: float) -> Dict[str, Any]:
    endpoint = base_url.rstrip("/") + "/chat/completions"
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_request},
        ],
        "temperature": 0,
        "max_tokens": 160,
        "response_format": {"type": "json_object"},
    }
    try:
        return post_json(endpoint, payload, api_key, timeout)
    except urllib.error.HTTPError as exc:
        # Some OpenAI-compatible servers do not implement response_format.
        if exc.code not in {400, 422}:
            raise
        payload.pop("response_format", None)
        return post_json(endpoint, payload, api_key, timeout)


def post_json(endpoint: str, payload: Dict[str, Any], api_key: str, timeout: float) -> Dict[str, Any]:
    headers = {"Content-Type": "application/json"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"

    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read().decode("utf-8")
    return json.loads(body)


def command_json_from_response(response: Dict[str, Any]) -> Dict[str, Any]:
    try:
        content = response["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as exc:
        raise CommandError("LLM response did not contain choices[0].message.content.") from exc
    return extract_json_object(content)


def extract_json_object(content: str) -> Dict[str, Any]:
    content = content.strip()
    if content.startswith("```"):
        content = re.sub(r"^```(?:json)?\s*", "", content)
        content = re.sub(r"\s*```$", "", content)
        content = content.strip()
    if not content.startswith("{"):
        match = re.search(r"\{.*\}", content, flags=re.DOTALL)
        if not match:
            raise CommandError(f"LLM did not return a JSON object: {content!r}")
        content = match.group(0)
    try:
        data = json.loads(content)
    except json.JSONDecodeError as exc:
        raise CommandError(f"LLM returned invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise CommandError("LLM JSON output must be an object.")
    return data


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.show_prompt:
        print(SYSTEM_PROMPT)
        return 0

    user_request = " ".join(args.request).strip()
    if not user_request:
        print("Usage: llm_command_bridge.py [--dry-run] 'move half a meter forward'")
        return 2

    safety = safety_from_args(args)
    api_key = os.environ.get(args.api_key_env, "")

    try:
        response = chat_completion(args.base_url, args.model, api_key, user_request, args.llm_timeout)
        command_json = command_json_from_response(response)
        command = parse_json_command(command_json, safety)
    except (CommandError, urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError) as exc:
        print(f"Rejected: {exc}")
        return 1

    print(f"User request: {user_request}")
    print(f"K2 JSON: {json.dumps(command_json, sort_keys=True)}")
    print(f"Accepted: {describe_command(command)}")

    if args.dry_run:
        print("Dry run only. Nothing published.")
        return 0

    rospy.init_node("ridgeback_llm_command_bridge", anonymous=False)
    executor = TextCommandExecutor(args, safety)
    try:
        executor.run_command(command)
    except CommandError as exc:
        executor.stop_robot()
        print(f"Rejected: {exc}")
        return 1
    finally:
        executor.stop_robot()
    return 0


if __name__ == "__main__":
    sys.exit(main())
