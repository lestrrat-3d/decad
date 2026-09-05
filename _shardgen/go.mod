// Module for the race-shard packer. It is nested and underscore-prefixed, so
// the root module never sees it: `go list ./...`, the linter and the test
// suite all skip it, exactly as they skip _gallery. Tooling does not belong in
// the library's own module.
module github.com/lestrrat-3d/decad/_shardgen

go 1.26.1
