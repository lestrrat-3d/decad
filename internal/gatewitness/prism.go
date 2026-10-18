// Package gatewitness selects payload prisms whose recorded wall stations
// provide a lower-bound diameter reference for Verify.
package gatewitness

import (
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Prism builds the straight prism the root fallback diameter reads its
// witnesses off, beside the displacement each of those witnesses can carry
// from the point of the denoted body it stands for. ok is false for every
// payload with no arm here, including a revolve payload (already exact through
// the root clearance carrier model, which is why it never reaches this fallback)
// and an analytic-walled prism payload whose section is its own denotation
// (the same reason). A free-form-walled prism payload's own section is its
// denotation too, so this switch answers false for it exactly as it does for
// the analytic case: root bodyGateDiameter routes a free-form-walled prism
// through diameter.PrismFreeformSection before its fallback, and calls this
// function only when that arm has already declined. This false answer then
// leaves the body with no gate diameter at all.
//
// The three arms read different geometry and earn a witness for different
// reasons. A capBlend payload reads one witness prism per loop instead
// (capblend.WitnessPrisms, through Prisms).
//
// A cup payload reads its view's outer, the cup's own outer region — the
// receiver's unmodified section for an INWARD shell, but the wider OFFSET
// (expanded) region for an OUTWARD one, since an outward shell adds material
// and cupPayloadFor (shell_cup.go) always assigns the wider of the two profiles
// to outer regardless of sense. Either way the whole cup body — walls, floor
// and cavity alike — sits inside the outer's own full-height prism: the cavity
// never reaches farther than the outer region, the same containment
// cupwall.Payload.ExtentAlong already relies on. Its outer walls run the cup's
// full height, so every station on them is a point of the body. The receiver's
// own section is its own denotation, because every modify op refuses a
// receiver carrying a section displacement (fillet.go's
// prismrecord.Payload.RequireExactSection), so an inward cup's witnesses carry
// only the axial displacement. An outward cup's outer region is the OFFSET
// one, recorded within the cup's offsetDelta of the region it denotes
// (shell_cup.go), so its witnesses carry that displacement beside the axial
// one, composed the way the displaced-prism arm below composes its own.
//
// A prism payload whose own sectionDelta is nonzero (docs/prism-boolean-design.md
// §7's re-expressed or cut section — every analytic Union whose merge cut a
// wall, plus any placed prism pair) reads its OWN recorded section, and is not
// a containing shape at all: the denoted section may sit either side of the
// recorded one. It does not need to be. What this gate needs is a lower bound
// on the body's own diameter, and §7 proves every recorded boundary point sits
// within sectionDelta of the section the payload denotes, while each recorded
// level sits within AxialDelta of the level it denotes. Those two displacements
// are perpendicular — one moves a coordinate IN the plane, the other moves a
// level ALONG the normal — so their sum is an upper bound on how far a lifted
// witness sits from the denoted body point below it, and
// diameter.LowerForDisplacement turns the held maximum into the lower bound
// the gate wants. The copy zeroes sectionDelta because addPrismFaces
// (clearance_geom.go) refuses a displaced section outright: it builds the
// clearance kernel's certificate carriers, which have to be exact statements
// about a boundary, and a witness set for a diameter is neither a certificate
// nor a carrier.
func Prism(payload topo.Payload) (prismrecord.Payload, float64, bool) {
	switch pl := payload.(type) {
	case cupwall.Payload:
		cup := pl.View()
		witness := cup.OuterPrism()
		witness.Profile = cup.Outer
		displacement := proofbound.AbsSumUpper(witness.SectionDelta, witness.AxialDelta())
		witness.SectionDelta = 0
		return witness, displacement, true
	case prismrecord.Payload:
		if pl.SectionDelta == 0 {
			return prismrecord.Payload{}, 0, false
		}
		displacement := proofbound.AbsSumUpper(pl.SectionDelta, pl.AxialDelta())
		witness := pl
		witness.SectionDelta = 0
		return witness, displacement, true
	case sweeprecord.Payload:
		witness := pl.Prism
		displacement := proofbound.AbsSumUpper(witness.SectionDelta, witness.AxialDelta())
		witness.SectionDelta = 0
		return witness, displacement, true
	default:
		return prismrecord.Payload{}, 0, false
	}
}

// Prisms is Prism's reading for every payload. A stacked prism reads one
// witness prism per outer run (stackedrecord.Payload.OuterRuns) over that run's
// own interval. Every run's outer loop is the outer wall of the slabs it spans,
// so every witness is a point of the body. The first slab's outer swept over
// the whole stack would hold points a union-built stack does not reach, and
// would overstate its diameter. A capblend payload reads capblend.WitnessPrisms.
func Prisms(payload topo.Payload) ([]prismrecord.Payload, float64, bool) {
	if pl, ok := payload.(capblend.Payload); ok {
		return capblend.WitnessPrisms(pl), pl.AxialDelta(), true
	}
	if pl, ok := payload.(stackedrecord.Payload); ok {
		runs := pl.OuterRuns()
		for i := range runs {
			runs[i].SectionDelta = 0
		}
		return runs, proofbound.AbsSumUpper(pl.SectionDelta, pl.AxialDelta()), true
	}
	witness, displacement, ok := Prism(payload)
	if !ok {
		return nil, 0, false
	}
	return []prismrecord.Payload{witness}, displacement, true
}
