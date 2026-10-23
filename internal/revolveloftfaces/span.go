package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

type arcSeg = sectionrecord.ArcSeg

type loftFaceInterval = loftface.Interval

type loftFacesTrimRecord = facetedrecord.LoftFacesTrimRecord

type pointSectionRecord = facetedrecord.PointSectionRecord

type revolveSectionSpan = facetedrecord.RevolveSectionSpan

// revolveLoftFacesMappedCell is a strict side or one transverse crossing of
// a complete source Bezier span over one LoftFaces band. A crossing's two
// cap brackets are exact rational enclosures, not rounded held vertices.
type RevolveLoftFacesMappedCell struct {
	Segment, Span, Band int
	Side                int
	JointKind           RevolveLoftFacesJointKind
	JointPoint          *sketch.Point
	JointArc            *sketch.Arc
	First, Last         loftface.RootCrossing
	XFirst, XLast       loftFaceInterval
	XDirection          int
	SLo, SHi            *big.Rat
	TLo, THi            *big.Rat
	TLoOpen, THiOpen    bool
}
