// Module for the Lean proofs' trace checker. It is nested and underscore-prefixed, so
// the root module never sees it: `go list ./...`, the linter and the test
// suite all skip it, exactly as they skip _gallery and _shardgen. It imports
// only the standard library; the Lake project beside it pins Lean and Mathlib in
// lake-manifest.json, and neither is a Go dependency.
module github.com/lestrrat-3d/decad/_proofs

go 1.27.4
