package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// LoftCylinderBoreRecord names the two certified annular caps of an unplaced
// through-bore result. Each outer and inner loop is CCW when viewed from +Z.
type LoftCylinderBoreRecord struct {
	Blank *topo.Body
	// Teeth are exact Cut sources; ProofTeeth and AxialBridge retain the
	// private Union witnesses for later source-level admission checks.
	Teeth, ProofTeeth                   []*topo.Body
	AxialBridge                         []float64
	Outer, Inner                        [2][]int
	CapGroups                           [2]int
	WallGroup                           int
	BlankRadius, BoreRadius, BoreBound  float64
	InnerInset, OuterExcess, RootCollar float64
	Z0, Z1                              float64
}

// LoftCylinderPlacement keeps the local source bodies used by the structural
// certificate. The public operands and their producer identities stay in the
// surrounding join record. A later bore rebuilds the local join from these
// sources instead of retaining its mesh. Only this exact common pose maps the
// local result to those operands; an ordinary later placement drops the record.
type LoftCylinderPlacement struct {
	Pose  r3.Transform
	Blank *topo.Body
	Teeth []*topo.Body
}

// A cylinder joined to a matching axial Loft keeps the original operands.
// Subsequent placements of that same Loft rebuild the common cylinder ring.
type LoftCylinderJoinRecord struct {
	Blank     *topo.Body
	Placement *LoftCylinderPlacement
	// Teeth are the exact Union operands. ProofTeeth are private, axially
	// normalized witnesses whose mesh proofs include AxialBridge.
	Teeth, ProofTeeth []*topo.Body
	AxialBridge       []float64
	CapOutlines       [2][]int
	CapGroups         [2]int
	MinSideRadius     float64
	Radius            float64
	// These upward bounds enclose the replaced root seam around Radius.
	// Retained tooth exterior walls still read their Loft source bounds.
	// The symmetric collar covers source-root exceptions near either cap.
	InnerInset, OuterExcess, RootCollar float64
	Z0, Z1                              float64
}

// The original LoftFaces proof authenticates this map over Source.SourceBox.
// A later Boolean may use it only after proving that its far cap survived.
type LoftFacesCapRecord struct {
	Source       *PointSectionRecord
	Maps         []loftface.Map
	Bounds       []LoftFaceCapBounds
	StationGroup []int
	Final        loftface.Map
	Group        int
	// Tangent is set only when the last section is a constructed
	// cone-tangent end. It is nil for plain LoftFaces.
	Tangent *loftface.TangentEnd
}

// This record is issued to one unplaced Cut of the original LoftFaces mesh.
// Placement drops it, and the following Intersect does not copy it.
type LoftFacesToeRecord struct {
	Base       Payload
	Cap        *LoftFacesCapRecord
	CapFace    *topo.Face
	ToeCapFace *topo.Face
	Toe        *PointConeLimit
	ToeTool    *topo.Body
}

// LoftFacesTrimRecord retains the authenticated source of one unplaced
// LoftFaces body after its original toe Cut and heel Intersect.
// The Boolean result remains faceted; this record only admits later
// source-specific proofs after those source identities are checked again.
type LoftFacesTrimRecord struct {
	Base                      Payload
	Source                    *PointSectionRecord
	Maps                      []loftface.Map
	Bounds                    []LoftFaceCapBounds
	StationGroup              []int
	Toe, Heel                 *PointConeLimit
	ToeTool, HeelTool         *topo.Body
	ToeCapGroup, HeelCapGroup int
	// Tangent is the cone-tangent end of the lofted tooth, copied from its
	// cap record. It is nil unless the loft ended in a constructed section.
	Tangent *loftface.TangentEnd
}

type LoftFaceCapBounds struct {
	StationRound        float64
	ProjectiveDeparture []float64
	PolygonSpeed        []float64
	SourceSpeed         []float64
	SourceDeparture     []float64
}

// This record retains the certified private lower bore and the full shell's
// exterior and bore loops for a later outer-cap rewrite. Placement clears it.
type LoftMirrorCylinderBoreRecord struct {
	Join       *LoftMirrorCylinderJoinRecord
	LowerBore  *topo.Body
	CapGroups  [2]int
	Outer      [2][]int
	Inner      [2][]int
	WallGroups [2]int
}

// A mirrored cylinder Join rebuilds the lower source against one exact half
// of the original blank. The upper source is its authenticated reflection.
// The original Boolean operands remain recorded for each later pattern step.
type LoftMirrorCylinderJoinRecord struct {
	Blank        *topo.Body
	Source       *LoftMirrorJoinRecord
	Pairs        []*topo.Body
	LowerHalf    *topo.Body
	CapGroups    [2]int
	CapOutlines  [2][]int
	BridgeBound  float64
	BridgeVolume float64
}

// LoftMirrorJoinRecord ties a sewn three-ring mesh back to two authenticated
// Lofts. The lower source occupies ring 0..1, its reflected copy ring 1..2.
// Wall groups stay distinct even when both sources use the same role string.
type LoftMirrorJoinRecord struct {
	Lower, Upper *topo.Body
	StationCount int
	WallGroups   [2][]int
	CapGroups    [2]int
}

// A rigid copy keeps the source relation separate from its moved held mesh.
// The source record is never reauthenticated from a faceted surface.
type LoftMirrorPlacementRecord struct {
	Source *LoftMirrorJoinRecord
	Motion r3.Transform
}
