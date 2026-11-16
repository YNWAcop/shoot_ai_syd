from __future__ import print_function
import os
import sys
import shlex
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
from desktop_tabs import terminal_args


class DesktopTabsTests(unittest.TestCase):
    def test_eight_tabs_and_active_competition(self):
        args = terminal_args('/tmp/syd', '/tmp/log', [])
        self.assertEqual(args.count('--window'), 1)
        self.assertEqual(args.count('--tab'), 7)
        self.assertEqual(args.count('-e'), 8)
        self.assertIn('--title=syd | Competition - enter 1', args)
        commands = [shlex.split(args[i+1]) for i,x in enumerate(args) if x == '-e']
        self.assertEqual(sum('watch' in c for c in commands), 7)
        self.assertEqual(sum('run' in c for c in commands), 1)

    def test_arguments_preserved_without_shell_expansion(self):
        race = ['set:=mission.rehearsal_pause_s=0; shooting.ring.rings.aim_x_px=350.5',
                'dry_fire:=true', 'literal:=$(not_a_command)']
        args = terminal_args('/tmp/syd project', '/tmp/log dir', race)
        command = shlex.split(args[-1])
        self.assertEqual(command[-3:], race)
        self.assertIn('/tmp/syd project', command)
        self.assertIn('/tmp/log dir', command)


if __name__ == '__main__':
    unittest.main()
