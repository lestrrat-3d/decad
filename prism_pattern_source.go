package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/r3"
)

// prismPatternSourceOf checks only recorded Extrude sections and Copy
// placements. The first tool and its copies must share the very same source
// record; a profile merely similar to that source cannot enter this path.
func prismPatternSourceOf(ctx context.Context, operands []*Body) (*prismPatternSource, bool, error) {
	if len(operands) < 3 || len(operands) > 33 {
		return nil, false, nil
	}
	blank, ok := operands[0].n.Payload.(prismPayload)
	if !ok || len(blank.Profile.Holes) != 0 || len(blank.Profile.Outer.Segments) != 1 ||
		blank.Xform != r3.Identity() {
		return nil, false, nil
	}
	circle, ok := blank.Profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (planePoint{}) || circle.Radius.Base() <= 0 ||
		!prismpattern.WholeRange(circle.TStart, circle.TEnd) {
		return nil, false, nil
	}
	first, ok := operands[1].n.Payload.(prismPayload)
	if !ok || first.Frame != blank.Frame || first.Z0 != blank.Z0 || first.Z1 != blank.Z1 ||
		first.Xform != r3.Identity() || len(first.Profile.Holes) != 0 ||
		len(first.Profile.Outer.Segments) < 4 {
		return nil, false, nil
	}
	segments := first.Profile.Outer.Segments
	var root arcSeg
	rootIndex := -1
	for i, segment := range segments {
		arc, isArc := segment.(arcSeg)
		if !isArc || arc.Center != circle.Center || !prismpattern.WholeRange(arc.TStart, arc.TEnd) {
			continue
		}
		gap, good := prismpattern.ArcGap(arc, circle.Radius.Base(), r3.Identity())
		if !good || gap >= 0.01 {
			continue
		}
		if rootIndex >= 0 {
			return nil, true, fmt.Errorf("%w: copied prism section has multiple possible circular joins", ErrUnsupported)
		}
		root, rootIndex = arc, i
	}
	if rootIndex < 0 {
		return nil, false, nil
	}
	gap := 0.0
	leastScale2 := big.NewRat(1, 1)
	blankRadiusSq := prismpattern.RadiusSq(circle.Radius.Base())
	for i, body := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		section, isPrism := body.n.Payload.(prismPayload)
		if !isPrism || section.Frame != first.Frame || section.Z0 != first.Z0 ||
			section.Z1 != first.Z1 || !prismpattern.AxialRotation(section.Xform) ||
			len(section.Profile.Outer.Segments) != len(segments) ||
			&section.Profile.Outer.Segments[0] != &segments[0] {
			return nil, true, fmt.Errorf("%w: section %d is not a radial copy of the first prism", ErrUnsupported, i+1)
		}
		localGap, good := prismpattern.ArcGap(root, circle.Radius.Base(), section.Xform)
		if !good {
			return nil, true, fmt.Errorf("%w: section %d has no finite circular-root bound", ErrUnsupported, i+1)
		}
		gap = max(gap, localGap)
		scale2 := prismpattern.Scale2(section.Xform)
		if scale2.Sign() <= 0 {
			return nil, true, fmt.Errorf("%w: section %d has no positive radial placement scale", ErrUnsupported, i+1)
		}
		if !prismpattern.RootInside(root, blankRadiusSq, scale2) {
			return nil, true, fmt.Errorf("%w: section %d circular root is not strictly inside the blank",
				ErrUnsupported, i+1)
		}
		if scale2.Cmp(leastScale2) < 0 {
			leastScale2 = scale2
		}
	}
	if gap >= 0.01 {
		return nil, true, fmt.Errorf("%w: copied prism root arcs exceed the circular-wall bound", ErrUnsupported)
	}
	// The exact dyadic placement can differ from a unit rotation by one or
	// more ulps. Prove every non-root source outside the blank after the
	// smallest radial scale among all copies, not only in source coordinates.
	outsideRadiusSq := new(big.Rat).Quo(blankRadiusSq, leastScale2)
	if err := prismpattern.OutsideRoot(ctx, segments, rootIndex, root, outsideRadiusSq); err != nil {
		return nil, true, err
	}
	return &prismPatternSource{Blank: blank, BlankRadius: circle.Radius.Base(), Section: first,
		RootArc: root, RootIndex: rootIndex,
		RootGap: gap, Sections: bodyNodes(operands[1:])}, true, nil
}
