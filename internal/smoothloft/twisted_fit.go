package smoothloft

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// MaxTwistedSections is §13's ceiling on the number of sections.
const MaxTwistedSections = 64

// twistSection is one section's edges in the first section's cyclic order:
// edges[a][s] is span s of edge a.
type twistSection struct {
	edges [][]twistSpan
}

func twistUnsupported(format string, args ...any) error {
	return fmt.Errorf("%w: "+format, append([]any{decaderr.ErrUnsupported}, args...)...)
}

// twistFitIndex finds the single fit spline of a loop and checks that the
// remaining records are whole lines.
func twistFitIndex(profile momentinput.Profile) (int, error) {
	segments := profile.Outer.Segments
	if len(profile.Holes) != 0 || len(segments) < 3 || len(segments) > 9 {
		return 0, twistUnsupported("a twisted loft section needs one hole-free loop of one fit spline and 2 to 8 lines")
	}
	fit := -1
	for j, raw := range segments {
		switch seg := raw.(type) {
		case sectionrecord.FitSplineSeg:
			if fit >= 0 {
				return 0, twistUnsupported("a twisted loft section has more than one fit spline")
			}
			if !((seg.TStart == 0 && seg.TEnd == 1) || (seg.TStart == 1 && seg.TEnd == 0)) {
				return 0, twistUnsupported("a twisted loft cannot use a trimmed fit spline")
			}
			fit = j
		case sectionrecord.LineSeg:
			if !((seg.TStart == 0 && seg.TEnd == 1) || (seg.TStart == 1 && seg.TEnd == 0)) {
				return 0, twistUnsupported("a twisted loft cannot use a trimmed line")
			}
		default:
			return 0, twistUnsupported("a twisted loft section needs only whole lines and one fit spline")
		}
	}
	if fit < 0 {
		return 0, twistUnsupported("a twisted loft section needs one fit spline")
	}
	return fit, nil
}

func twistRat(p sectionrecord.Point2) freeform.RatPoint {
	return freeform.RatPoint{U: rf(p.U), V: rf(p.V)}
}

func twistSame(a, b freeform.RatPoint) bool { return a.U.Cmp(b.U) == 0 && a.V.Cmp(b.V) == 0 }

// twistGather resolves every section to exact cubic spans, aligned to the
// first section's cyclic order by the fit spline, and checks the exact joins.
func twistGather(profiles []momentinput.Profile, work []*freeform.FreeformWork) ([]twistSection, error) {
	sections := make([]twistSection, len(profiles))
	var fitAt0, count int
	var spanCount []int
	for i, profile := range profiles {
		fit, err := twistFitIndex(profile)
		if err != nil {
			return nil, err
		}
		m := len(profile.Outer.Segments)
		if i == 0 {
			fitAt0, count = fit, m
		}
		if m != count {
			return nil, twistUnsupported("twisted loft sections have different edge counts")
		}
		walks, err := momentinput.ResolveProfileWalks(profile, work[i])
		if err != nil {
			return nil, err
		}
		edges := make([][]twistSpan, m)
		for a := range m {
			orig := ((fit+a-fitAt0)%m + m) % m
			switch seg := profile.Outer.Segments[orig].(type) {
			case sectionrecord.LineSeg:
				start, end := twistRat(seg.Start), twistRat(seg.End)
				if seg.TStart == 1 {
					start, end = end, start
				}
				edges[a] = []twistSpan{twistLineSpan(start, end)}
			case sectionrecord.FitSplineSeg:
				spans := fitWalkSpans(walks.At(0, orig))
				if len(spans) == 0 || len(spans) > 32 {
					return nil, twistUnsupported("a twisted loft fit spline needs 1 to 32 curve spans")
				}
				edges[a] = make([]twistSpan, len(spans))
				for s, span := range spans {
					if len(span) != 4 {
						return nil, twistUnsupported("a twisted loft fit spline needs cubic Bézier spans")
					}
					copy(edges[a][s][:], span)
				}
			}
		}
		if i == 0 {
			spanCount = make([]int, m)
			for a := range edges {
				spanCount[a] = len(edges[a])
			}
		}
		for a := range edges {
			if len(edges[a]) != spanCount[a] {
				return nil, twistUnsupported("twisted loft sections have different fit-point counts or record kinds")
			}
		}
		for a := range edges {
			last := edges[a][len(edges[a])-1][3]
			if !twistSame(last, edges[(a+1)%m][0][0]) {
				return nil, twistUnsupported("a twisted loft section has unequal recorded joins")
			}
			for s := 1; s < len(edges[a]); s++ {
				if !twistSame(edges[a][s-1][3], edges[a][s][0]) {
					return nil, twistUnsupported("a twisted loft fit spline has disjoint spans")
				}
			}
		}
		sections[i] = twistSection{edges: edges}
	}
	return sections, nil
}

// twistPlanes checks the shared world-XY frame and strictly ascending Z, and
// returns the exact levels.
func twistPlanes(planes []sectionrecord.PlaneRecord) ([]*big.Rat, error) {
	zs := make([]*big.Rat, len(planes))
	for i := range planes {
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return nil, twistUnsupported("twisted loft planes must share world-XY axes and origin")
		}
		zs[i] = rf(planes[i].Origin.Z)
		if i > 0 && zs[i].Cmp(zs[i-1]) <= 0 {
			return nil, twistUnsupported("twisted loft planes need strictly ascending Z levels")
		}
	}
	return zs, nil
}

// twistPatches builds and proves every wall patch. The result is indexed
// [span][height span], where span runs over all edge spans in edge order.
func twistPatches(ctx context.Context, sections []twistSection, zs []*big.Rat) ([][]twistPatch, []int, error) {
	n := len(sections)
	var edgeOf []int
	var patches [][]twistPatch
	budget := proofbound.NewWorkBudget(ctx)
	for a := range sections[0].edges {
		for s := range sections[0].edges[a] {
			// tangents[i][j] is section i's Z slope of control point j.
			tangents := make([]twistSpan, n)
			for j := range 4 {
				points := make([]freeform.RatPoint, n)
				for i := range sections {
					points[i] = sections[i].edges[a][s][j]
				}
				for i, m := range twistSlopes(points, zs) {
					tangents[i][j] = m
				}
			}
			row := make([]twistPatch, n-1)
			for i := range row {
				if err := ctx.Err(); err != nil {
					return nil, nil, err
				}
				row[i] = twistPatchOf(sections[i].edges[a][s], sections[i+1].edges[a][s], tangents[i], tangents[i+1], sub(zs[i+1], zs[i]))
				if err := row[i].provePolarPositive(ctx, budget); err != nil {
					return nil, nil, err
				}
			}
			patches = append(patches, row)
			edgeOf = append(edgeOf, a)
		}
	}
	return patches, edgeOf, nil
}

// BuildTwistedFits constructs agent-docs/loft-sections-design.md §13's loft:
// three to MaxTwistedSections sections of one fit spline and whole lines
// that may turn about the plane origin.
func BuildTwistedFits(ctx context.Context, profiles []momentinput.Profile,
	planes []sectionrecord.PlaneRecord, work []*freeform.FreeformWork) (Build, error) {
	if len(profiles) < 3 || len(profiles) > MaxTwistedSections ||
		len(planes) != len(profiles) || len(work) != len(profiles) {
		return Build{}, twistUnsupported("twisted loft needs 3 to %d matching sections", MaxTwistedSections)
	}
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	zs, err := twistPlanes(planes)
	if err != nil {
		return Build{}, err
	}
	sections, err := twistGather(profiles, work)
	if err != nil {
		return Build{}, err
	}
	patches, edgeOf, err := twistPatches(ctx, sections, zs)
	if err != nil {
		return Build{}, err
	}
	return buildTwistedMesh(ctx, patches, edgeOf, len(sections[0].edges), planes, zs)
}
