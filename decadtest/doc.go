// Package decadtest is the kit a decad test author calls instead of reading
// decad's struct definitions directly. Every helper takes testing.TB first,
// calls tb.Helper(), and stops the test with tb.Fatalf on failure rather than
// returning an error for the caller to check.
//
// The comparison rule: a decad.Measurement is a proven interval [V-B, V+B],
// and the author's want is a second claim W ± s, where s is the error of the
// author's own oracle. The two claims agree exactly when |V - W| <= B + s. B
// is always read from the reading itself, never supplied by the test.
//
// decadtest is production code, not test code: it never imports testify,
// because doing so would make testify a non-test dependency of the root
// module, which CLAUDE.md's approved-module list forbids. Only decadtest's
// own _test.go files, in package decadtest_test, import
// github.com/stretchr/testify/require.
//
// The kit never reads a mesh. Body.Tessellate, STL and OBJ stay uncovered so
// that no test built on this kit grows a dependency on triangle structure
// (docs/api-design.md §3 invariant 1).
//
// It also runs decad.Document.Verify for the test and asserts on the
// returned report: its own status, its per-body records, its diagnostic and
// pair inventories, and the three per-body surveys (wall, undercut,
// concave radius).
//
// This file states the whole contract; there is no separate design document
// for callers to read.
//
// The kit has no examples/ entry. An Example* function has no testing.TB to
// hand a helper, so it could not demonstrate a single call. This package's
// own doc comments and its tests are the usage record instead.
package decadtest
