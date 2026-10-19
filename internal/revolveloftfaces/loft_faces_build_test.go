package revolveloftfaces_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// loftFacesFixture is a one-segment point-loft source whose base fan has
// three ordered sides from the apex and one cap triangle, and two selected
// sections over its three-station ring.
func loftFacesFixture() ([]revolveloftfaces.LoftFacesSection, *facetedrecord.PointSectionRecord) {
	base := facetedrecord.Payload{
		Tris:   [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 2, 3}},
		Src:    []int{0, 0, 0, 1},
		Groups: make([]facetedrecord.Group, 2),
	}
	slice := &facetedrecord.PointPlaneSliceRecord{Base: base}
	capMesh := facetedrecord.Payload{Groups: []facetedrecord.Group{
		{Origins: []provenance.Ref{{Producer: 2, Role: provenance.RoleCapEnd}}}}}
	ring := func(z float64) []r3.Vec { return []r3.Vec{{X: 1, Z: z}, {Y: 1, Z: z}, {X: -1, Z: z}} }
	sections := []revolveloftfaces.LoftFacesSection{
		{Mesh: capMesh, Slice: slice, Ring: ring(0)},
		{Mesh: capMesh, Slice: slice, Ring: ring(1)},
	}
	source := &facetedrecord.PointSectionRecord{Profile: momentinput.Profile{
		Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{}}}}}
	return sections, source
}

func TestBuildLoftFacesRefusesMalformedSources(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 7, Alloc: topotest.Alloc{}}
	withBase := func(edit func(base *facetedrecord.Payload)) func([]revolveloftfaces.LoftFacesSection) {
		return func(sections []revolveloftfaces.LoftFacesSection) {
			slice := *sections[0].Slice
			slice.Base.Tris = append([][3]int(nil), slice.Base.Tris...)
			slice.Base.Src = append([]int(nil), slice.Base.Src...)
			edit(&slice.Base)
			for i := range sections {
				sections[i].Slice = &slice
			}
		}
	}
	cases := []struct {
		name string
		edit func([]revolveloftfaces.LoftFacesSection)
		want string
	}{
		{name: "base groups do not match the profile", want: "no complete side correspondence",
			edit: withBase(func(base *facetedrecord.Payload) { base.Groups = base.Groups[:1] })},
		{name: "sections with different stations", want: "section 1 has different stations",
			edit: func(sections []revolveloftfaces.LoftFacesSection) { sections[1].Ring = sections[1].Ring[:2] }},
		{name: "side not from the apex", want: "no ordered side",
			edit: withBase(func(base *facetedrecord.Payload) { base.Tris[1] = [3]int{1, 2, 3} })},
		{name: "side outside the profile", want: "source side has no segment",
			edit: withBase(func(base *facetedrecord.Payload) { base.Src[2] = 1 })},
		{name: "cap through the apex", want: "source cap includes the apex",
			edit: withBase(func(base *facetedrecord.Payload) { base.Tris[3] = [3]int{1, 2, 0} })},
		{name: "constructed end without a plane frame", want: "constructed loft end has no plane frame",
			edit: func(sections []revolveloftfaces.LoftFacesSection) {
				zero := loftface.RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
				sections[1] = revolveloftfaces.LoftFacesSection{Slice: sections[0].Slice, Ring: sections[1].Ring,
					Constructed: &loftface.Map{Constant: zero, QU: zero, QV: zero}}
			}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sections, source := loftFacesFixture()
			tc.edit(sections)
			body, err := revolveloftfaces.BuildLoftFaces(t.Context(), env, sections, source, nil)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
			require.Nil(t, body)
		})
	}
}
