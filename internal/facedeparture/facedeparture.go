// Package facedeparture lets the STEP writer (export) read whether a face's
// surface tag departs from the surface the face carries by a stated bound.
// The root package installs Reader when it loads; export calls Stated. It
// adds no public API.
package facedeparture

// Reader reports whether a *decad.Face states a nonzero normal departure from
// its tag (topo.Face.NormalBound or topo.Face.HeldNormalBound). Any other
// value, which it cannot read, reports true. The root package sets it once at
// load time.
var Reader func(face any) bool

// Stated reports whether face states a nonzero departure from its tag. Before
// the root package has installed Reader it reports true, so a reader that
// only refuses on it refuses every face rather than admitting one unread.
func Stated(face any) bool {
	if Reader == nil {
		return true
	}
	return Reader(face)
}
