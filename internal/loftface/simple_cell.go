package loftface

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// A simple source cell is one whole line, one exact fitted Bezier span, or
// one whole short arc in the authenticated Sketch boundary walk. Its box
// encloses the entire true curve, not a sampled polyline.
type SimpleCell struct {
	Segment, Span int
	Start, End    freeform.RatPoint
	Box           [2]Interval
	Bezier        freeform.BezierSpan
	Arc           *sectionrecord.ArcSeg
}
