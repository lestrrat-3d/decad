package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// CrossCylinderBoreRecord keeps the two full circular prisms whose Cut made
// one perpendicular through-bore. The mesh Cut owns its original contact
// proof; this record permits a later mouth chamfer to rebuild from the exact
// source circles instead of interpreting a sampled intersection as a curve.
type CrossCylinderBoreRecord struct {
	Target, Tool    *topo.Body
	Radius, Bore    float64
	Height, CenterZ float64
	SourceBound     float64
}

// JoinedMesh collects source-labeled facets for structural joins and rewrites.
// Each caller supplies its own geometric admission and boundary proofs.
type JoinedMesh struct {
	Verts  []r3.Vec
	Tris   [][3]int
	Src    []int
	Groups []Group
}

func (m *JoinedMesh) Add(t [3]int, group int) {
	m.Tris = append(m.Tris, t)
	m.Src = append(m.Src, group)
}

// ObliqueBossRecord is a one-use witness for a whole plate-to-cylinder
// junction. The generic Union still proves its own mesh and contact.
type ObliqueBossRecord struct {
	Plate, Boss  *topo.Body
	Rect         momentinput.AxisRect
	Bottom, Top  float64
	Base         r3.Vec
	Axis, Ex, Ey r3.Vec
	Radius       float64
	Length       float64
	SourceBound  float64
}

// PrismPatternSource is a bounded source relation for a circular prism and
// rigid copies of one adjacent prism section. It does not prove a Union:
// the shared wall and the caps still need a constructed boundary and volume
// proof before any result can be published.
type PrismPatternSource struct {
	Blank       prismrecord.Payload
	BlankRadius float64
	Section     prismrecord.Payload
	RootArc     sectionrecord.ArcSeg
	RootIndex   int
	RootGap     float64
	Sections    []*topo.Body
}

// PrismPatternPart is one authenticated source extrusion and its verified
// boundary mesh. The root wall starts at RootStart and ends at sample zero;
// the preceding samples form the exterior section path.
type PrismPatternPart struct {
	Body      *topo.Body
	Prism     prismrecord.Payload
	Mesh      *topo.Mesh
	RootStart int
	Start     float64
	End       float64
	StartIV   proofbound.RatInterval
	EndIV     proofbound.RatInterval
	Vertex    [][]int
}

// The Union keeps its authenticated sources and complete held shell so a
// later operation can prove a local rewrite without inferring source curves
// from faceted edges. These slices are immutable after the Union commits.
type PrismPatternJoinRecord struct {
	Blank     *topo.Body
	Source    *PrismPatternSource
	Parts     []PrismPatternPart
	Outlines  [2][]int
	Groups    [3]int
	MaxStep   float64
	RingRound float64
	Mesh      *JoinedMesh
}

// PrismUnionFilletRecord keeps the two immutable sources of a mesh Union.
// Only an original Union gets this record; placement drops it. The sources
// may be retired, so every use authenticates the selected result edge and
// works through booleanBody without committing an intermediate body.
type PrismUnionFilletRecord struct{ First, Second *topo.Body }
