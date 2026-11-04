#!/usr/bin/python3
"""course.yaml の値を1つ表示する(launch の $(command ...) から使う)。

  course_value.py course.yaml start.x        → 0.5
  course_value.py course.yaml start.yaw 0    → 値がなければ 0
"""
import sys

import yaml


def main():
    path, key = sys.argv[1], sys.argv[2]
    default = sys.argv[3] if len(sys.argv) > 3 else None
    v = yaml.safe_load(open(path))
    for k in key.split("."):
        if not isinstance(v, dict) or k not in v:
            if default is None:
                sys.exit(f"{key} not found in {path}")
            v = default
            break
        v = v[k]
    # 数値は必ず小数で出す(ROS のパラメータが double のところに整数 "0" を渡すとエラーになる)
    try:
        v = float(v)
        print(repr(v), end="")
    except (TypeError, ValueError):
        print(v, end="")


if __name__ == "__main__":
    main()
