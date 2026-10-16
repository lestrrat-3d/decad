package sectionclip

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// The section record functions read these evaluator records under the names
// the root package gives them.
type (
	profileRecord  = momentinput.Profile
	loopRecord     = sectionrecord.LoopRecord
	curveSegment   = sectionrecord.CurveSegment
	lineSeg        = sectionrecord.LineSeg
	prismPayload   = prismrecord.Payload
	loftPayload    = loftrecord.Payload
	facetedPayload = facetedrecord.Payload
)
