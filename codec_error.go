package decad

import (
	"errors"
	"fmt"
	"strings"
)

// codecPathError carries a path relative to the value currently being
// decoded. Nested validation prepends its field or array position.
type codecPathError struct {
	path string
	err  error
}

func (e *codecPathError) Error() string {
	if e.path == "" {
		return e.err.Error()
	}
	return fmt.Sprintf(`%s: %s`, e.path, e.err)
}

func (e *codecPathError) Unwrap() error { return e.err }

func joinCodecPath(prefix, suffix string) string {
	switch {
	case prefix == "":
		return suffix
	case suffix == "":
		return prefix
	case strings.HasPrefix(suffix, "["):
		return prefix + suffix
	default:
		return prefix + "." + suffix
	}
}

func prependCodecPath(err error, prefix string) error {
	if err == nil {
		return nil
	}
	var pathErr *codecPathError
	if errors.As(err, &pathErr) {
		return &codecPathError{
			path: joinCodecPath(prefix, pathErr.path),
			err:  pathErr.err,
		}
	}
	return &codecPathError{path: prefix, err: err}
}
