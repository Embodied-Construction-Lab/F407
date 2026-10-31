"""Check the explicit Windows build mode against the production safety header."""
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]


class CubeIDEConfigTest(unittest.TestCase):
    def test_external_profile_is_explicit_and_default_profiles_remain_disabled(self):
        root = ET.parse(PROJECT / '.cproject').getroot()
        configs = {c.attrib['name']: c for c in root.findall('.//configuration')}
        self.assertEqual(set(configs), {'Debug', 'Release', 'Release_ExternalCutoff'})
        for name, config in configs.items():
            with self.subTest(configuration=name):
                compiler = config.find(".//tool[@superClass='com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler']")
                symbols = compiler.find("option[@valueType='definedSymbols']")
                defines = [v.attrib['value'] for v in symbols.findall('listOptionValue')]
                expected = int(name == 'Release_ExternalCutoff')
                source = ('#include "board_safety.h"\n'
                          '#if BOARD_ESTOP_INPUT_ENABLED != 0\n#error unexpected GPIO mode\n#endif\n'
                          f'#if BOARD_EXTERNAL_POWER_CUTOFF_ENABLED != {expected}\n'
                          '#error wrong external cutoff mode\n#endif\n')
                subprocess.run(['cc', '-E', '-x', 'c', '-I', str(PROJECT / 'Core/Inc'),
                                *['-D' + s for s in defines], '-'], input=source,
                               text=True, check=True, stdout=subprocess.DEVNULL)
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
