package decadtest

import (
	"testing"

	"github.com/lestrrat-3d/decad"
)

// IsManifold checks that body's boundary is a closed manifold skin,
// mirroring decad's own soundness rule (verify.go's auditBoundary) through
// the public API alone: body MUST have at least one face; every face MUST
// have at least one loop, unless its surface is a full sphere or a full
// torus, which bounds a solid with no boundary loop at all; every loop MUST
// have at least one edge; and every edge MUST bound exactly two faces. body
// MUST NOT be nil.
//
// The last rule's failure branch has no producer through decad's public API
// today: every exported operation that builds a Body leaves each edge
// bounding exactly two faces, and Edge, Face and Loop carry only unexported
// fields, so no test outside package decad can construct a body that fails
// it. IsManifold's positive tests over a seamless cylindrical wall (two
// loops on one face) and a full-revolve torus (no loops at all) stand in for
// that branch: those are exactly the shapes an "every face has one outer
// loop" rule would have rejected.
func IsManifold(tb testing.TB, body *decad.Body) {
	tb.Helper()

	if body == nil {
		tb.Fatalf("decadtest.IsManifold: body must not be nil")
		return
	}

	faces := body.Faces()
	if len(faces) == 0 {
		tb.Fatalf("decadtest.IsManifold: body has no faces, want at least one")
		return
	}

	for fi, f := range faces {
		loops := f.Loops()
		if len(loops) == 0 {
			switch f.Surface().Kind() {
			case decad.KindSphere, decad.KindTorus:
				continue
			default:
				tb.Fatalf("decadtest.IsManifold: face %d (surface kind %v) has no loops, want at least one", fi, f.Surface().Kind())
				return
			}
		}
		for li, l := range loops {
			if len(l.Edges()) == 0 {
				tb.Fatalf("decadtest.IsManifold: face %d loop %d has no edges, want at least one", fi, li)
				return
			}
		}
	}

	for ei, e := range body.Edges() {
		if got := len(e.Faces()); got != 2 {
			tb.Fatalf("decadtest.IsManifold: edge %d bounds %d face(s), want exactly 2", ei, got)
			return
		}
	}
}
