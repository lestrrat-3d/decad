package facetedrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type SpiralCombineSource struct {
	First  *SpiralFirstJoinSource
	Tools  []*topo.Body
	Moves  []r3.Transform
	Copies *FacetedCopySet
}

type SpiralCombinedTools struct {
	Source *SpiralCombineSource
	// Owner maps each transient composite face to the live face it copies.
	Owner map[*topo.Face]*topo.Face
}

// SpiralHeelRootSeam encloses the true crossing of the source root arc and
// the blank's finite heel outer cone. Endpoints follow ArcSeg Start, End order.
// The intervals include the source arc's recorded-radius uncertainty.
type SpiralHeelRootSeam struct {
	X, Slope loftface.Interval
	Ends     [2][3]loftface.Interval
	ConeC    *big.Rat
	Negative bool
}

// SpiralBlankSource is the legacy six-side view of one authenticated Revolve
// section. The generic certificate owns the source walk, faces, and cone gap;
// these named corners only map the older spiral join's fixed cap roles.
type SpiralBlankSource struct {
	Section           RevolveSectionCertificate
	ToeAxis, HeelAxis sectionrecord.Point2
	ToeOuter, ToeRoot sectionrecord.Point2
	Ded, HeelOuter    sectionrecord.Point2
	EdgeIndex         [6]int
	RootFace          *topo.Face
}

// A bounded box around one true flank/root/tool triple junction. The true
// point has X=the corresponding exact ToeX or HeelX; s and t remain enclosed
// separately because their algebraic roots need not be rational.
type SpiralJunction struct {
	Band     int
	SLo, SHi *big.Rat
	TLo, THi *big.Rat
	Box      [3]loftface.Interval
}

// A first curved tooth join may use this only while both input bodies remain
// the original unplaced blank and cone-trimmed LoftFaces tooth. The matcher
// below rechecks the live Sketch source and the exact finite blank meridian.
type SpiralFirstJoinSource struct {
	Blank      *SpiralBlankSource
	Tooth      *LoftFacesTrimRecord
	RootM      *big.Rat
	RootB      *big.Rat
	ToeX       *big.Rat
	HeelX      *big.Rat
	ToeOffset  *big.Rat
	HeelOffset *big.Rat
	ToeCapGap  *big.Rat
	HeelCapGap *big.Rat
	Flank      [2]freeform.BezierSpan // root-containing cubic of groups 0 and 2
	Crossings  [2][9]loftface.RootCrossing
	Junctions  [2][2]SpiralJunction // flank 0/2, then toe/heel
	HeelSeam   *SpiralHeelRootSeam
}
