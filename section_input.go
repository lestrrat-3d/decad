package decad

import "github.com/lestrrat-3d/sketch"

// Section is the sketch geometry a section-fed operation consumes: a closed
// [*sketch.Profile] or an open [*sketch.Chain]. An operation declared as
// func Op[S Section](..., sec S, ...) infers S from the argument, so callers
// pass either kind without naming the type argument. The operation decides
// what each kind builds and which options it accepts; a kind or option the
// operation cannot honour returns ErrUnsupported, and a nil section returns
// ErrDegenerate.
type Section interface {
	*sketch.Profile | *sketch.Chain
}
