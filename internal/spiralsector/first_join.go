package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// CertifyFirstJoin proves that each of the two fitted flanks crosses
// the blank's recorded root meridian once and transversely through all eight
// continuous LoftFaces bands. It also proves that both root branches move
// strictly from toe to heel. The finite toe/heel transitions are retained as
// signed exact offsets for the cap-strip proof; they are not collapsed to a
// falsely shared ring.
//
// sections are the blank's candidate Revolve root sections; the blank source
// is the first one ExactBlank accepts.
func CertifyFirstJoin(ctx context.Context, sections []facetedrecord.RevolveSectionCertificate,
	tooth *topo.Body) (*spiralFirstJoinSource, bool, error) {
	if ctx == nil || tooth == nil {
		return nil, false, nil
	}
	section, ok := ExactBlank(sections)
	if !ok {
		return nil, false, nil
	}
	p, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok || p.LoftFacesTrim == nil || p.Xform != r3.Identity() {
		return nil, false, nil
	}
	trim := p.LoftFacesTrim
	if !TrimSourceCurrent(trim) || !facetedtopology.CapGroupsPresent(p.Src, p.FaceOf, len(p.Groups),
		trim.ToeCapGroup, trim.HeelCapGroup) {
		return nil, false, nil
	}
	m, b := section.Section.MeridianEquation()
	if !SourcePhysicalSector(trim, m, b) {
		return nil, false, nil
	}
	if !OtherSides(ctx, trim, m, b) {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		return nil, false, nil
	}
	toeX, ok := RootToolX(m, b, trim.Toe)
	if !ok {
		return nil, false, nil
	}
	heelX, ok := RootToolX(m, b, trim.Heel)
	if !ok || toeX.Cmp(heelX) >= 0 ||
		seamRatAdd(seamRatMul(m, toeX), b).Sign() <= 0 ||
		seamRatAdd(seamRatMul(m, heelX), b).Sign() <= 0 {
		return nil, false, nil
	}
	toeGap, ok := ToolOuterGap(trim.Toe, section.ToeRoot, section.ToeOuter, 1)
	if !ok {
		return nil, false, nil
	}
	heelGap, ok := ToolOuterGap(trim.Heel, section.Ded, section.HeelOuter, -1)
	if !ok {
		return nil, false, nil
	}
	proof := &spiralFirstJoinSource{Blank: section, Tooth: trim, RootM: m, RootB: b,
		ToeX: toeX, HeelX: heelX,
		ToeOffset:  seamRatSub(toeX, seamRatF(section.ToeRoot.U)),
		HeelOffset: seamRatSub(heelX, seamRatF(section.Ded.U)),
		ToeCapGap:  toeGap, HeelCapGap: heelGap}
	// The real finite root lies between the tool-clipped ends, with a small
	// overhang at each cap. The later cap proof charges both exact offsets.
	if proof.ToeOffset.Sign() >= 0 || proof.HeelOffset.Sign() <= 0 {
		return nil, false, nil
	}
	if !OriginalCapsAvoidToolRootCircles(trim, m, b, toeX, heelX) &&
		!TangentCapTouchesHeelCircle(trim, m, b, toeX, heelX) {
		return nil, false, nil
	}
	for side, group := range [...]int{0, 2} {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		fit, ok := trim.Source.Profile.Outer.Segments[group].(sectionrecord.FitSplineSeg)
		if !ok {
			return nil, false, nil
		}
		work := freeform.NewFreeformWork()
		work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		spans, err := splinebezier.FitSplineBezierSpans(fit, work)
		if err != nil {
			return nil, false, err
		}
		if len(spans) != 14 {
			return nil, false, nil
		}
		proof.Flank[side] = spans[0]
		for band := range 8 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			for spanIndex, span := range spans {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				q := loftface.MeridianQRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
				if spanIndex == 0 {
					if !polynomial.BernsteinRowsCross(q) ||
						!loftface.CapXDecreases(trim.Maps[band], span) ||
						!loftface.CapXDecreases(trim.Maps[band+1], span) ||
						!polynomial.BernsteinRowsPositive(loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, q)) {
						return nil, false, nil
					}
					if band == 0 {
						proof.Crossings[side][0] = loftface.IsolateRoot(q[0])
					}
					proof.Crossings[side][band+1] = loftface.IsolateRoot(q[2])
				} else if !polynomial.BernsteinRowsPositive(q) {
					return nil, false, nil
				}
			}
		}
		toeRange := loftface.CapXRange(trim.Maps[0], spans[0], proof.Crossings[side][0])
		heelRange := loftface.CapXRange(trim.Maps[8], spans[0], proof.Crossings[side][8])
		if toeRange.Hi.Cmp(toeX) >= 0 || heelRange.Lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
		for end, targetX := range []*big.Rat{toeX, heelX} {
			junction, ok := IsolateJunction(ctx, trim.Maps, spans[0], m, b,
				proof.Crossings[side], targetX)
			if !ok {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				return nil, false, nil
			}
			proof.Junctions[side][end] = junction
		}
	}
	// The embedded source solid's boundary meets each tool/root circle only
	// at these two flank junctions. All other contour groups stay on one side.
	// The whole original cap 0 is below toeX. The whole cap-8 plane either
	// misses its heel tool/root circle (the strict branch), or, for a
	// cone-tangent end with a matching record, touches that circle at one
	// point with the circle on the source side of the plane
	// (TangentCapTouchesHeelCircle). Either way neither end-cap
	// interior can create an extra contact, even though cap 8 spans the
	// heelX coordinate.
	// Q_s and J make both junctions transverse. Circle membership therefore
	// changes at each junction. The long connecting arc crosses Y<0, where
	// the entire source is absent, so the short arc is inside the source.
	// The ordered exact Cut and Intersect define the operative tool-cone cap
	// as this source solid's trace on that cone. The retained cap group IDs
	// identify held facets only; they do not establish the true trace.
	seam, ok, err := CertifySpiralHeelRootSeam(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	proof.HeelSeam = seam
	return proof, true, nil
}
