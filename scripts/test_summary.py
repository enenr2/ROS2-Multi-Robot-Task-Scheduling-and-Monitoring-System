#!/usr/bin/env python3
"""把 colcon 测试结果汇总成 Markdown（CI 写进 GITHUB_STEP_SUMMARY）。

只读 build/*/test_results 下的 gtest/xunit XML，不依赖 colcon 的汇总口径，
避免把 CTest 包装层的测试再算一遍。
"""
import glob
import sys
import xml.etree.ElementTree as ET


def main():
    rows = {}
    for path in sorted(glob.glob('build/*/test_results/*/*.xml')):
        package = path.split('/')[1]
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue
        entry = rows.setdefault(package, [0, 0, 0])
        entry[0] += int(root.get('tests', 0))
        entry[1] += int(root.get('failures', 0))
        entry[2] += int(root.get('errors', 0))

    if not rows:
        print('没有找到测试结果（build/*/test_results）')
        return 0

    total = [0, 0, 0]
    print('| 包 | 用例 | 失败 | 错误 |')
    print('|---|---:|---:|---:|')
    for package, (tests, failures, errors) in sorted(rows.items()):
        print(f'| `{package}` | {tests} | {failures} | {errors} |')
        total = [a + b for a, b in zip(total, (tests, failures, errors))]

    status = '通过' if total[1] == 0 and total[2] == 0 else '失败'
    print(f'\n**合计 {total[0]} 项，{total[1] + total[2]} 失败 —— {status}**')
    return 0


if __name__ == '__main__':
    sys.exit(main())