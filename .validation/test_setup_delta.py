import json
from pathlib import Path
import tempfile
import unittest
import validation_controller as ctrl

class SetupEvidence(unittest.TestCase):
    def test_wrapper_output_is_separated_from_go_events(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as root:
            log=Path(root)/'test.jsonl'
            log.write_text('Compiler storage before stage\nFilesystem bytes\n'+json.dumps({'Action':'pass','Test':'TestExample'})+'\n')
            ctrl.verify_tests({'tests':['TestExample']},log)
            self.assertEqual(json.loads(log.with_suffix('.go.jsonl').read_text())['Test'],'TestExample')
            self.assertIn('Filesystem bytes',log.with_suffix('.wrapper.log').read_text())
    def test_oversized_module_evidence_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as root:
            root=Path(root);(root/'evidence').mkdir();(root/'artifacts').mkdir()
            (root/'evidence'/'resolved-modules.json').write_bytes(b'x'*((256<<10)+1))
            with self.assertRaisesRegex(RuntimeError,'limits'):
                ctrl.collect_setup_evidence(root,root/'artifacts',{})
