// Package apitest holds decad's tests that call only the exported API. It has
// no exported identifiers; its _test.go files are package apitest_test and
// import github.com/lestrrat-3d/decad like any other caller.
//
// The tests live outside the root package so that the root test binary builds
// faster. An external test file in the root directory compiles only after the
// root package and its internal tests have compiled, one after the other.
// Here the tests compile beside them, against the root package alone.
//
// A test that reads unexported state belongs in the root package as an
// internal (package decad) test. A test that reads repository files by a
// path relative to the root, and the Example functions godoc shows with the
// package, stay in the root package too.
package apitest
