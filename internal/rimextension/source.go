package rimextension

import (
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// BoundFace ties a retained face to the native face of its producer. The
// translation accumulates only committed geometry-changing tool motions.
type BoundFace struct {
	Final, Native *topo.Face
	Body          *topo.Body
	Translation   r3.Vec
}

// Admission contains native-domain evidence for a proposed held chart.
type Admission struct {
	Section                Section
	Setback                float64
	Stock                  *topo.Body
	Cap, Side, Boss, Round BoundFace
	Stop                   []BoundFace
	Allowed                map[int]bool
}

// WindowBounds encloses source-coordinate displacement, source area, and
// removed true volume in the admitted window.
type WindowBounds struct {
	Deviation, Area, Volume float64
	support                 SourceWindow
}
