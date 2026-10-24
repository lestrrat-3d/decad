package revolveloftfaces_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func rat(n int64) *big.Rat { return big.NewRat(n, 1) }

func TestRefFaceIndex(t *testing.T) {
	t.Parallel()

	ref := provenance.Ref{Producer: 2, Role: "side(0,1)"}
	other := provenance.Ref{Producer: 2, Role: "side(0,2)"}
	base := func() facetedrecord.Payload {
		return facetedrecord.Payload{
			Tris:   [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}},
			Src:    []int{0, 0, 1},
			FaceOf: []int{2, 2, 5},
			Groups: []facetedrecord.Group{{Origins: []provenance.Ref{ref}}, {Origins: []provenance.Ref{other}}},
		}
	}
	index, ok := revolveloftfaces.RefFaceIndex(base(), ref, 6)
	require.True(t, ok)
	require.Equal(t, 2, index)
	index, ok = revolveloftfaces.RefFaceIndex(base(), other, 6)
	require.True(t, ok)
	require.Equal(t, 5, index)
	index, ok = revolveloftfaces.RefFaceIndex(base(), provenance.Ref{Producer: 9}, 6)
	require.True(t, ok, "no face carries the role")
	require.Equal(t, -1, index)

	tests := map[string]func(*facetedrecord.Payload){
		"source count differs":  func(p *facetedrecord.Payload) { p.Src = p.Src[:2] },
		"face count differs":    func(p *facetedrecord.Payload) { p.FaceOf = p.FaceOf[:2] },
		"group below range":     func(p *facetedrecord.Payload) { p.Src[0] = -1 },
		"group above range":     func(p *facetedrecord.Payload) { p.Src[2] = 2 },
		"face below range":      func(p *facetedrecord.Payload) { p.FaceOf[0] = -1 },
		"face above range":      func(p *facetedrecord.Payload) { p.FaceOf[0], p.FaceOf[1] = 6, 6 },
		"two faces for one ref": func(p *facetedrecord.Payload) { p.FaceOf[1] = 3 },
	}
	for name, edit := range tests {
		p := base()
		edit(&p)
		index, ok := revolveloftfaces.RefFaceIndex(p, ref, 6)
		require.False(t, ok, name)
		require.Equal(t, -1, index, name)
	}
}

func TestCompleteArcRun(t *testing.T) {
	t.Parallel()

	trimOf := func(groups ...int) *facetedrecord.LoftFacesTrimRecord {
		source := &facetedrecord.PointSectionRecord{SourceStation: make([]sectionrecord.Point2, len(groups))}
		source.Profile.Outer.Segments = make([]sectionrecord.CurveSegment, 3)
		return &facetedrecord.LoftFacesTrimRecord{Source: source, StationGroup: groups}
	}
	tests := []struct {
		name    string
		trim    *facetedrecord.LoftFacesTrimRecord
		segment int
		want    []int
	}{
		{"a middle arc", trimOf(0, 0, 1, 1, 2, 2), 1, []int{2, 3, 4}},
		{"the first arc", trimOf(0, 0, 1, 1, 2, 2), 0, []int{0, 1, 2}},
		{"an arc that wraps the seam", trimOf(1, 2, 2, 0, 0, 1), 1, []int{5, 0, 1}},
		{"a single chord", trimOf(0, 1, 2), 1, []int{1, 2}},
	}
	for _, tc := range tests {
		got, ok := revolveloftfaces.CompleteArcRun(tc.trim, tc.segment)
		require.True(t, ok, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}

	refusals := []struct {
		name    string
		trim    *facetedrecord.LoftFacesTrimRecord
		segment int
	}{
		{"no trim", nil, 0},
		{"no source", &facetedrecord.LoftFacesTrimRecord{StationGroup: []int{0, 1, 2}}, 0},
		{"negative segment", trimOf(0, 1, 2), -1},
		{"segment past the profile", trimOf(0, 1, 2), 3},
		{"station count differs", func() *facetedrecord.LoftFacesTrimRecord {
			trim := trimOf(0, 1, 2)
			trim.StationGroup = []int{0, 1, 2, 2}
			return trim
		}(), 0},
		{"fewer than three stations", func() *facetedrecord.LoftFacesTrimRecord {
			trim := trimOf(0, 1)
			return trim
		}(), 0},
		{"two runs", trimOf(0, 1, 0, 2), 0},
		{"no run", trimOf(0, 0, 2, 2), 1},
		{"one arc covers every chord", trimOf(1, 1, 1), 1},
	}
	for _, tc := range refusals {
		got, ok := revolveloftfaces.CompleteArcRun(tc.trim, tc.segment)
		require.False(t, ok, tc.name)
		require.Nil(t, got, tc.name)
	}
}

// The comparisons below have no input that tells a mutant apart: a negative
// segment or one past the profile finds no run, a run over every chord is also
// no run, an upper limit below zero implies a lower one, a limit equal to the
// other is refused before the miss test, and a limit sign change always has
// both signs.
func TestOriginsMatch(t *testing.T) {
	t.Parallel()

	a, b := provenance.Ref{Producer: 1, Role: "a"}, provenance.Ref{Producer: 1, Role: "b"}
	require.True(t, revolveloftfaces.OriginsMatch(nil, nil))
	require.True(t, revolveloftfaces.OriginsMatch([]provenance.Ref{a, b}, []provenance.Ref{a, b}))
	require.False(t, revolveloftfaces.OriginsMatch([]provenance.Ref{a}, []provenance.Ref{a, b}))
	require.False(t, revolveloftfaces.OriginsMatch([]provenance.Ref{a, b}, []provenance.Ref{b, a}))
	require.False(t, revolveloftfaces.OriginsMatch([]provenance.Ref{a, b}, []provenance.Ref{a, {Producer: 2, Role: "b"}}))
}

func TestToolBoundaryOutward(t *testing.T) {
	t.Parallel()

	rising := loftface.VerticalLimit{Before: -1, After: 1}
	falling := loftface.VerticalLimit{Before: 1, After: -1}
	tests := []struct {
		name        string
		edge        sketch.BoundaryEdge
		limit       loftface.VerticalLimit
		outward, ok bool
	}{
		{"rising forward", sketch.BoundaryEdge{}, rising, true, true},
		{"falling forward", sketch.BoundaryEdge{}, falling, false, true},
		{"rising reversed", sketch.BoundaryEdge{Reversed: true}, rising, false, true},
		{"falling reversed", sketch.BoundaryEdge{Reversed: true}, falling, true, true},
		{"a partial fragment", sketch.BoundaryEdge{Partial: true}, rising, false, false},
		{"no sign change", sketch.BoundaryEdge{}, loftface.VerticalLimit{Before: 1, After: 1}, false, false},
		{"an undecided side", sketch.BoundaryEdge{}, loftface.VerticalLimit{}, false, false},
		{"a double sign change", sketch.BoundaryEdge{}, loftface.VerticalLimit{Before: -2, After: 1}, false, false},
	}
	for _, tc := range tests {
		outward, ok := revolveloftfaces.ToolBoundaryOutward(tc.edge, tc.limit)
		require.Equal(t, tc.ok, ok, tc.name)
		require.Equal(t, tc.outward, outward, tc.name)
	}
}

func TestSameCrossing(t *testing.T) {
	t.Parallel()

	base := loftface.RootCrossing{Span: 1, Lo: rat(1), Hi: rat(2)}
	require.True(t, revolveloftfaces.SameCrossing(base, loftface.RootCrossing{Span: 1, Lo: rat(1), Hi: rat(2)}))
	require.False(t, revolveloftfaces.SameCrossing(base, loftface.RootCrossing{Span: 2, Lo: rat(1), Hi: rat(2)}))
	require.False(t, revolveloftfaces.SameCrossing(base, loftface.RootCrossing{Span: 1, Lo: rat(0), Hi: rat(2)}))
	require.False(t, revolveloftfaces.SameCrossing(base, loftface.RootCrossing{Span: 1, Lo: rat(1), Hi: rat(3)}))
	require.False(t, revolveloftfaces.SameCrossing(base, loftface.RootCrossing{Span: 1, Lo: rat(1)}))
}

func mappedCell() revolveloftfaces.RevolveLoftFacesMappedCell {
	return revolveloftfaces.RevolveLoftFacesMappedCell{
		Segment: 1, Span: 2, Band: 3, Side: revolveloftfaces.RevolveLoftFacesJointContact,
		JointKind:  revolveloftfaces.RevolveLoftFacesWholeArcContact,
		First:      loftface.RootCrossing{Span: 1, Lo: rat(1), Hi: rat(2)},
		Last:       loftface.RootCrossing{Span: 2, Lo: rat(3), Hi: rat(4)},
		XFirst:     loftface.Interval{Lo: rat(5), Hi: rat(6)},
		XLast:      loftface.Interval{Lo: rat(7), Hi: rat(8)},
		XDirection: 1,
		SLo:        rat(0), SHi: rat(1), TLo: rat(2), THi: rat(3),
	}
}

func TestSameMappedCell(t *testing.T) {
	t.Parallel()

	require.True(t, revolveloftfaces.SameMappedCell(mappedCell(), mappedCell()))
	point, arc := new(sketch.Point), new(sketch.Arc)
	edits := map[string]func(*revolveloftfaces.RevolveLoftFacesMappedCell){
		"segment":     func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Segment++ },
		"span":        func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Span++ },
		"band":        func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Band++ },
		"side":        func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Side++ },
		"joint kind":  func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.JointKind++ },
		"point":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.JointPoint = point },
		"arc":         func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.JointArc = arc },
		"first":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.First.Span++ },
		"last":        func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Last.Hi = rat(9) },
		"x first":     func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.XFirst.Lo = rat(9) },
		"x last":      func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.XLast.Hi = rat(9) },
		"x direction": func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.XDirection = -1 },
		"s low":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.SLo = rat(9) },
		"s high":      func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.SHi = rat(9) },
		"t low":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.TLo = rat(9) },
		"t high":      func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.THi = rat(9) },
		"t low open":  func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.TLoOpen = true },
		"t high open": func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.THiOpen = true },
	}
	for name, edit := range edits {
		changed := mappedCell()
		edit(&changed)
		require.False(t, revolveloftfaces.SameMappedCell(mappedCell(), changed), name)
	}
}

func TestSameArcContact(t *testing.T) {
	t.Parallel()

	require.True(t, revolveloftfaces.SameArcContact(mappedCell(), mappedCell()))
	point, arc := new(sketch.Point), new(sketch.Arc)
	edits := map[string]func(*revolveloftfaces.RevolveLoftFacesMappedCell){
		"segment": func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Segment++ },
		"span":    func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Span++ },
		"band":    func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Band++ },
		"side":    func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.Side = 1 },
		"joint kind": func(c *revolveloftfaces.RevolveLoftFacesMappedCell) {
			c.JointKind = revolveloftfaces.RevolveLoftFacesFitLineContact
		},
		"point":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.JointPoint = point },
		"arc":         func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.JointArc = arc },
		"s low":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.SLo = rat(9) },
		"s high":      func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.SHi = rat(9) },
		"t low":       func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.TLo = rat(9) },
		"t high":      func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.THi = rat(9) },
		"t low open":  func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.TLoOpen = true },
		"t high open": func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.THiOpen = true },
		"no s low":    func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.SLo = nil },
		"no s high":   func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.SHi = nil },
		"no t low":    func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.TLo = nil },
		"no t high":   func(c *revolveloftfaces.RevolveLoftFacesMappedCell) { c.THi = nil },
	}
	for name, edit := range edits {
		changed := mappedCell()
		edit(&changed)
		require.False(t, revolveloftfaces.SameArcContact(mappedCell(), changed), name)
		require.False(t, revolveloftfaces.SameArcContact(changed, mappedCell()), name)
	}
	other := mappedCell()
	other.Side = 1
	require.False(t, revolveloftfaces.SameArcContact(other, other), "only a joint contact side counts")
	other = mappedCell()
	other.JointKind = revolveloftfaces.RevolveLoftFacesFitLineContact
	require.False(t, revolveloftfaces.SameArcContact(other, other), "only a whole-arc contact counts")
}

func TestVerticalLimits(t *testing.T) {
	t.Parallel()

	trim := func() *facetedrecord.LoftFacesTrimRecord {
		return &facetedrecord.LoftFacesTrimRecord{
			Toe:  &facetedrecord.PointConeLimit{Axis: coneaxis.X(), Apex: rat(2), Far: rat(-5), Slope: rat(1)},
			Heel: &facetedrecord.PointConeLimit{Axis: coneaxis.X(), Apex: rat(10), Far: rat(-5), Slope: rat(2)},
		}
	}
	span := func(from, to float64) facetedrecord.RevolveSectionSpan {
		return facetedrecord.RevolveSectionSpan{
			Start: sectionrecord.Point2{V: from}, End: sectionrecord.Point2{V: to}}
	}

	tests := []struct {
		name         string
		x            *big.Rat
		section      facetedrecord.RevolveSectionSpan
		lower, upper *big.Rat
		outcome      revolveloftfaces.VerticalOutcome
	}{
		{"past the toe apex only the heel limits", rat(4), span(0, 5), rat(0), rat(3), revolveloftfaces.VerticalContact},
		{"before the toe apex both limit", rat(1), span(0, 5), rat(1), big.NewRat(9, 2), revolveloftfaces.VerticalContact},
		{"a reversed section is ordered", rat(4), span(5, 0), rat(0), rat(3), revolveloftfaces.VerticalContact},
		{"a section above the heel limit misses", rat(4), span(5, 6), rat(5), rat(3), revolveloftfaces.VerticalMiss},
		{"x on the toe apex keeps a negative lower", rat(2), span(-2, 1), rat(-2), rat(1), revolveloftfaces.VerticalRefusal},
		{"touching limits are undecided", rat(4), span(3, 5), rat(3), rat(3), revolveloftfaces.VerticalRefusal},
		{"a negative radial limit refuses", rat(4), span(-2, 1), rat(-2), rat(1), revolveloftfaces.VerticalRefusal},
	}
	for _, tc := range tests {
		lower, upper, outcome := revolveloftfaces.VerticalLimits(trim(), tc.section, tc.x)
		require.Equal(t, tc.outcome, outcome, tc.name)
		require.Zero(t, tc.lower.Cmp(lower), "%s: lower %v", tc.name, lower)
		require.Zero(t, tc.upper.Cmp(upper), "%s: upper %v", tc.name, upper)
	}

	refusals := map[string]func(*facetedrecord.LoftFacesTrimRecord) *big.Rat{
		"no toe":          func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Toe = nil; return rat(4) },
		"no heel":         func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Heel = nil; return rat(4) },
		"no x":            func(*facetedrecord.LoftFacesTrimRecord) *big.Rat { return nil },
		"no toe apex":     func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Toe.Apex = nil; return rat(4) },
		"no heel apex":    func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Heel.Apex = nil; return rat(4) },
		"no toe far":      func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Toe.Far = nil; return rat(4) },
		"no heel far":     func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Heel.Far = nil; return rat(4) },
		"no toe slope":    func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Toe.Slope = nil; return rat(4) },
		"no heel slope":   func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Heel.Slope = nil; return rat(4) },
		"flat toe slope":  func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Toe.Slope = rat(0); return rat(4) },
		"flat heel slope": func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat { r.Heel.Slope = rat(0); return rat(4) },
		"x on the toe far plane": func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat {
			r.Heel.Far = rat(-9)
			return rat(-5)
		},
		"x on the heel far plane": func(r *facetedrecord.LoftFacesTrimRecord) *big.Rat {
			r.Toe.Far = rat(-9)
			return rat(-5)
		},
		"x on the heel apex": func(*facetedrecord.LoftFacesTrimRecord) *big.Rat { return rat(10) },
	}
	for name, edit := range refusals {
		r := trim()
		x := edit(r)
		lower, upper, outcome := revolveloftfaces.VerticalLimits(r, span(0, 5), x)
		require.Equal(t, revolveloftfaces.VerticalRefusal, outcome, name)
		require.Nil(t, lower, name)
		require.Nil(t, upper, name)
	}
	lower, upper, outcome := revolveloftfaces.VerticalLimits(nil, span(0, 5), rat(4))
	require.Equal(t, revolveloftfaces.VerticalRefusal, outcome)
	require.Nil(t, lower)
	require.Nil(t, upper)
}
