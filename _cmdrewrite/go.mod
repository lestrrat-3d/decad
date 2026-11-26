// Module for the command-API call-site rewriter. It is nested and
// underscore-prefixed, so the root module never sees it: `go list ./...`, the
// linter and the test suite all skip it, exactly as they skip _gallery.
// Tooling does not belong in the library's own module.
module github.com/lestrrat-3d/decad/_cmdrewrite

go 1.27.4
