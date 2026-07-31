# Copyright 2025 Intelligent Robotics Lab
#
# This file is part of the project Easy Navigation (EasyNav in short)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
from typing import Dict, List, Set
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)

from ros2cli.verb import VerbExtension


BASE_CLASSES = {
    'mapsmanager': {'title': 'mapsmanager plugins', 'base': 'easynav::MapsManagerBase'},
    'localizer': {'title': 'localizer plugins', 'base': 'easynav::LocalizerMethodBase'},
    'planner': {'title': 'planner plugins', 'base': 'easynav::PlannerMethodBase'},
    'controller': {'title': 'controller plugins', 'base': 'easynav::ControllerMethodBase'},
    'costmap_filters': {'title': 'costmap filters', 'base': 'easynav::CostmapFilter'},
    'navmap_filters': {'title': 'navmap filters', 'base': 'easynav::navmap::NavMapFilter'},
    'sensors': {'title': 'sensor perception handlers', 'base': 'easynav::PerceptionHandler'},
}
CATEGORY_ORDER = ['mapsmanager', 'localizer', 'planner', 'controller', 'costmap_filters',
                  'navmap_filters', 'sensors']


def _ament_index_roots() -> List[str]:
    """Return all candidate '.../share/ament_index/resource_index' root across overlay + system."""
    roots: List[str] = []
    prefixes = list(get_packages_with_prefixes().values())
    seen: Set[str] = set()
    for p in prefixes:
        per_pkg = os.path.join(p, 'share', 'ament_index', 'resource_index')
        if os.path.isdir(per_pkg) and per_pkg not in seen:
            roots.append(per_pkg)
            seen.add(per_pkg)
        parent = os.path.dirname(p)  # .../install
        common = os.path.join(parent, 'share', 'ament_index', 'resource_index')
        if os.path.isdir(common) and common not in seen:
            roots.append(common)
            seen.add(common)
    return roots


def _iter_all_plugin_xml_paths(debug: bool = False) -> List[str]:
    """
    Collect all plugin XML paths from ALL ament index roots.

    Supports both key styles:
      - 'pluginlib__<something>'
      - '<category>__pluginlib__plugin'
    Resolves lines that start with 'share/' against the package PREFIX (not share),
    and other relative paths against the package SHARE dir.
    """
    results: List[str] = []
    roots = _ament_index_roots()
    if debug:
        print(f'[debug] ament index roots: {len(roots)}')
        for r in roots:
            print(f'[debug]   - {r}')

    def interesting(key: str) -> bool:
        return key.startswith('pluginlib__') or key.endswith('__pluginlib__plugin')

    # cache of pkg -> prefix/share
    pkgs_prefixes = get_packages_with_prefixes()
    share_cache: Dict[str, str] = {}

    for root in roots:
        try:
            keys = os.listdir(root)
        except Exception:
            continue
        for resource_key in keys:
            if not interesting(resource_key):
                continue
            key_dir = os.path.join(root, resource_key)
            if not os.path.isdir(key_dir):
                continue

            for pkg in os.listdir(key_dir):
                resource_file = os.path.join(key_dir, pkg)
                if not os.path.isfile(resource_file):
                    continue
                try:
                    with open(resource_file, 'r', encoding='utf-8') as f:
                        content = f.read()
                except Exception:
                    continue

                pkg_prefix = pkgs_prefixes.get(pkg)  # e.g. .../install/<pkg>
                for line in content.splitlines():
                    raw = line.strip()
                    if not raw:
                        continue
                    if os.path.isabs(raw):
                        results.append(raw)
                        continue

                    # If path starts with 'share/...', resolve from package prefix
                    if raw.startswith('share/') and pkg_prefix:
                        abs_path = os.path.normpath(os.path.join(pkg_prefix, raw))
                        results.append(abs_path)
                        continue

                    # Otherwise resolve from package share dir
                    try:
                        share_dir = share_cache.get(pkg)
                        if share_dir is None:
                            share_dir = get_package_share_directory(pkg)
                            share_cache[pkg] = share_dir
                        abs_path = os.path.normpath(os.path.join(share_dir, raw))
                        results.append(abs_path)
                    except Exception:
                        # As a last resort, keep raw
                        results.append(raw)

    # De-duplicate preserving order
    seen: Set[str] = set()
    unique: List[str] = []
    for p in results:
        if p not in seen:
            unique.append(p)
            seen.add(p)

    if debug:
        print(f'[debug] candidate XML files: {len(unique)}')
        for p in unique:
            print(f'[debug]   - {p}')
    return unique


def _parse_plugins(xml_path: str) -> List[Dict[str, str]]:
    """Parse one pluginlib XML and return all <class> entries with attributes."""
    try:
        tree = ET.parse(xml_path)
    except Exception:
        return []
    root = tree.getroot()
    out: List[Dict[str, str]] = []
    for lib in root.findall('library'):
        lib_path = lib.get('path', '')
        for cls in lib.findall('class'):
            out.append({
                'type': cls.get('type', ''),
                'name': cls.get('name', ''),
                'base_class_type': cls.get('base_class_type', ''),
                'library': lib_path,
                'description': (cls.findtext('description') or '').strip(),
                'xml': xml_path,
            })
    return out


def _print_plugins(plugins: List[Dict[str, str]], *, show_lib: bool, show_xml: bool):
    for p in plugins:
        line = f"- {p['name'] or p['type']}: type='{p['type']}"
        if show_lib and p.get('library'):
            line += f"  [lib: {p['library']}]"
        if show_xml and p.get('xml'):
            line += f"  [xml: {p['xml']}]"
        print(line)


class PluginsVerb(VerbExtension):
    """List EasyNav pluginlib plugins grouped by categories."""

    def add_arguments(self, parser, cli_name):
        # Category filters
        parser.add_argument('--mapsmanager', action='store_true',
                            help='Only show mapsmanager plugins.')
        parser.add_argument('--localizer', action='store_true',
                            help='Only show localizer plugins.')
        parser.add_argument('--planner', action='store_true',
                            help='Only show planner plugins.')
        parser.add_argument('--controller', action='store_true',
                            help='Only show controller plugins.')
        parser.add_argument('--costmap-filters', action='store_true',
                            help='Only show costmap filters.')
        parser.add_argument('--navmap-filters', action='store_true',
                            help='Only show navmap filters.')
        parser.add_argument('--sensors', action='store_true',
                            help='Only show sensor perception handler plugins.')

        # Output options
        parser.add_argument('--show-xml', action='store_true',
                            help='Show the XML path declaring the plugin.')
        parser.add_argument('--show-lib', action='store_true',
                            help='Show the <library> path if available.')
        parser.add_argument('--grep',
                            help='Filter by substring in plugin name or type.')

        # Machine-readable + debug
        parser.add_argument('--json', action='store_true',
                            help='Output JSON instead of text.')
        parser.add_argument('--pretty', action='store_true',
                            help='Pretty-print JSON (only with --json).')
        parser.add_argument('--debug', action='store_true',
                            help='Print debug info about ament index scanning.')

    def main(self, *, args):
        selected = {
            'mapsmanager': args.mapsmanager,
            'localizer': args.localizer,
            'planner': args.planner,
            'controller': args.controller,
            'costmap_filters': args.costmap_filters,
            'navmap_filters': args.navmap_filters,
            'sensors': args.sensors,
        }
        if not any(selected.values()):
            for k in selected:
                selected[k] = True

        all_xmls = _iter_all_plugin_xml_paths(debug=args.debug)
        all_classes: List[Dict[str, str]] = []
        for xml in all_xmls:
            all_classes.extend(_parse_plugins(xml))

        total = 0
        json_out: Dict[str, Dict] = {}

        for key in CATEGORY_ORDER:
            if not selected[key]:
                continue
            cfg = BASE_CLASSES[key]
            base = cfg['base']

            plugins = [c for c in all_classes if c['base_class_type'] == base]

            if args.grep:
                g = args.grep.lower()
                plugins = [p for p in plugins if g in (p['name'] or '').lower()
                           or g in (p['type'] or '').lower()]

            if not args.json:
                print(f"\n{cfg['title']}  (base: {base}) — {len(plugins)} found")
                _print_plugins(plugins, show_lib=args.show_lib, show_xml=args.show_xml)

            json_out[key] = {'title': cfg['title'], 'base': base, 'count': len(plugins),
                             'plugins': plugins}
            total += len(plugins)

        if args.json:
            payload = {'total': total, 'categories': json_out}
            print(json.dumps(payload, indent=2 if args.pretty else None, ensure_ascii=False))
        else:
            print(f'\nTotal plugins found: {total}')
        return 0
