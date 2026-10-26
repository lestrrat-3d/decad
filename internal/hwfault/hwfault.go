// Package hwfault holds the test guard for tests that deliberately provoke a
// hardware fault and recover it.
//
// A nil pointer dereference or an integer division by zero is not an explicit
// panic call. The CPU raises a fault, the runtime turns it into a panic, and
// the test recovers it. Several differential tests do this on purpose: they
// feed nil-field or malformed operands to an exact-arithmetic routine and
// compare the recovered panic with the one the older body raised.
//
// On Windows with Go 1.27, recovering a hardware fault is unsafe on hosts
// whose CPUs save a large extended register state (Intel AMX). Windows
// delivers the fault by writing the saved CPU context onto the goroutine's own
// stack, the context is larger than the space the runtime reserves for it, and
// the write can land in a neighbouring heap object. The corruption stays
// silent until later. Then the garbage collector or the allocator stops the
// whole test binary with "found pointer to free object" or "unknown caller
// pc", and the failing test is an innocent one that happened to be running.
// This is Go issue golang/go#81238, open at the time of writing; the long-term
// fix in the Go runtime is tracked in golang/go#47576. It shows up only on
// runners that happen to have such a CPU, which is why CI failed on Windows
// intermittently.
//
// The guarded tests keep running on Linux and macOS, where the same fault is a
// signal the runtime handles on a separate stack. Lift the guard when a Go
// release fixes the Windows exception path.
package hwfault

import (
	"runtime"
	"testing"
)

// skipper is the part of testing.TB that SkipOnWindows needs.
type skipper interface {
	Helper()
	Skip(args ...any)
}

// SkipOnWindows skips t on Windows. Call it first in a test that provokes and
// recovers a hardware fault; see the package comment for why.
func SkipOnWindows(t testing.TB) {
	t.Helper()
	skip(runtime.GOOS, t)
}

func skip(goos string, t skipper) {
	if goos == "windows" {
		t.Skip("recovering a hardware fault corrupts the heap on Windows with Go 1.27; see internal/hwfault")
	}
}
