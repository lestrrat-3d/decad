package prismcells

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func polygonProfile(corners ...[2]float64) momentinput.Profile {
	var p momentinput.Profile
	for i := range corners {
		a, b := corners[i], corners[(i+1)%len(corners)]
		p.Outer.Segments = append(p.Outer.Segments, sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: a[0], V: a[1]}, End: sectionrecord.Point2{U: b[0], V: b[1]}, TEnd: 1})
	}
	return p
}

// TestSectionWithinAndApart holds both readings to the arrangement: a rail
// whose corners lie on its longer stock's edges is within it, a rail past the
// stock's end is not, a wedge sharing part of the rail's top edge is apart
// from it, and a wedge sunk below that edge, or whose rounded corners leave the
// shared span only near-coincident, is not.
func TestSectionWithinAndApart(t *testing.T) {
	t.Parallel()
	stock := polygonProfile([2]float64{-332, 474}, [2]float64{292, 366}, [2]float64{292, 401}, [2]float64{-332, 509})
	rail := polygonProfile([2]float64{-280, 465}, [2]float64{240, 375}, [2]float64{240, 410}, [2]float64{-280, 500})
	long := polygonProfile([2]float64{-340, 475.384615}, [2]float64{240, 375}, [2]float64{240, 410}, [2]float64{-340, 510.384615})
	// The rail's top edge falls 9 for every 52 along it, so these corners lie
	// on it exactly.
	wedge := polygonProfile([2]float64{-176, 482}, [2]float64{-124, 473}, [2]float64{-124, 525}, [2]float64{-176, 525})
	sunk := polygonProfile([2]float64{-176, 481}, [2]float64{-124, 472}, [2]float64{-124, 525}, [2]float64{-176, 525})
	// lower(x) rounds, so this wedge's corners miss the rail's edge by an ulp
	// and the shared span is only near-coincident.
	lower := func(x float64) float64 { return 500 - (x+280)*90/520 }
	rounded := polygonProfile([2]float64{-195, lower(-195)}, [2]float64{-105, lower(-105)}, [2]float64{-105, 525}, [2]float64{-195, 525})
	budget := proofbound.NewWorkBudget(t.Context())
	for _, tc := range []struct {
		name string
		got  func() (bool, error)
		want bool
	}{
		{"rail within its stock", func() (bool, error) {
			return SectionWithin(t.Context(), budget, []momentinput.Profile{rail}, []momentinput.Profile{stock})
		}, true},
		{"longer rail not within", func() (bool, error) {
			return SectionWithin(t.Context(), budget, []momentinput.Profile{long}, []momentinput.Profile{stock})
		}, false},
		{"wedge on the rail apart", func() (bool, error) {
			return SectionsApart(t.Context(), budget, []momentinput.Profile{wedge}, []momentinput.Profile{rail})
		}, true},
		{"wedge on a rounded edge undecided", func() (bool, error) {
			return SectionsApart(t.Context(), budget, []momentinput.Profile{rounded}, []momentinput.Profile{rail})
		}, false},
		{"sunk wedge not apart", func() (bool, error) {
			return SectionsApart(t.Context(), budget, []momentinput.Profile{sunk}, []momentinput.Profile{rail})
		}, false},
	} {
		got, err := tc.got()
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
}
