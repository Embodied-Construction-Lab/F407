"""Check the ordinary Windows build configurations and project paths."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]


class CubeIDEConfigTest(unittest.TestCase):
    def test_ioc_uses_active_project_name(self):
        settings = dict(line.split('=', 1) for line in
                        (PROJECT / 'data_celect.ioc').read_text().splitlines()
                        if '=' in line and not line.startswith('#'))
        self.assertEqual(settings['ProjectManager.ProjectName'], 'data_celect')
        self.assertEqual(settings['ProjectManager.ProjectFileName'], 'data_celect.ioc')

    def test_standard_profiles_have_consistent_project_paths(self):
        root = ET.parse(PROJECT / '.cproject').getroot()
        configs = {c.attrib['name']: c for c in root.findall('.//configuration')}
        self.assertEqual(set(configs), {'Debug', 'Release'})
        for name, config in configs.items():
            with self.subTest(configuration=name):
                self.assertEqual(config.find('.//builder').attrib['buildPath'],
                                 '${workspace_loc:/data_celect}/' + name)
                self.assertEqual(config.attrib['artifactName'], '${ProjName}')
                self.assertEqual(config.find('folderInfo').attrib['id'],
                                 config.attrib['id'] + '.')
                entries = {x.attrib['name'] for x in config.findall('sourceEntries/entry')}
                self.assertEqual(entries, {'Core', 'Drivers'})
        # Per-configuration Eclipse instances must not collide when imported.
        ids = [node.attrib['id'] for c in root.findall('.//cconfiguration')
               for node in c.findall('.//*[@id]')
               if node.tag not in ('storageModule', 'extension')]
        self.assertEqual(len(ids), len(set(ids)))


if __name__ == '__main__':
    unittest.main()
