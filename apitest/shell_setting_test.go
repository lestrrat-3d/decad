package apitest_test

import (
	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// shellSetting is one method call, Sense or FaceThickness, that both
// ShellCommand and ShellClosedCommand take. Tables hold settings as values
// and applyShell or applyShellClosed turns them into method calls.
type shellSetting struct {
	open   func(*decad.ShellCommand) *decad.ShellCommand
	closed func(*decad.ShellClosedCommand) *decad.ShellClosedCommand
}

func senseSetting(s decad.ShellSense) shellSetting {
	return shellSetting{
		open:   func(c *decad.ShellCommand) *decad.ShellCommand { return c.Sense(s) },
		closed: func(c *decad.ShellClosedCommand) *decad.ShellClosedCommand { return c.Sense(s) },
	}
}

func outwardSetting() shellSetting { return senseSetting(decad.Outward) }

func faceThicknessSetting(normal r3.Vec, t units.Value) shellSetting {
	return shellSetting{
		open:   func(c *decad.ShellCommand) *decad.ShellCommand { return c.FaceThickness(normal, t) },
		closed: func(c *decad.ShellClosedCommand) *decad.ShellClosedCommand { return c.FaceThickness(normal, t) },
	}
}

// applyShell builds Shell(b, sel, t) and calls each setting's method on it.
func applyShell(b *decad.Body, sel decad.FaceSelector, t units.Value, settings ...shellSetting) *decad.ShellCommand {
	cmd := decad.Shell(b, sel, t)
	for _, s := range settings {
		cmd = s.open(cmd)
	}
	return cmd
}

// applyShellClosed builds ShellClosed(b, t) and calls each setting's method on it.
func applyShellClosed(b *decad.Body, t units.Value, settings ...shellSetting) *decad.ShellClosedCommand {
	cmd := decad.ShellClosed(b, t)
	for _, s := range settings {
		cmd = s.closed(cmd)
	}
	return cmd
}
