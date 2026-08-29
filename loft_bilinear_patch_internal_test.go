package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCellBilinearAreaEnclosesDirectIntegral(t *testing.T) {
	vLo := r3.NewVec(1, -2, 0.5)
	vHi := r3.NewVec(4, 0, 1)
	wLo := r3.NewVec(-0.5, 2, 6)
	wHi := r3.NewVec(3, 4, 7.5)

	value, bound := cellBilinearArea(vLo, vHi, wLo, wHi)
	reference := bilinearAreaMidpoint(vLo, vHi, wLo, wHi, 1024)
	require.False(t, math.IsInf(bound, 0), "the ordinary finite cell must have a finite area enclosure")
	require.Positive(t, bound, "the warped cell's integration interval must have nonzero width")
	require.LessOrEqual(t, math.Abs(reference-value), bound,
		"the certified bilinear-area interval must enclose an independent direct integral")
}

func TestCellTwistMomentsMatchRefinedBilinearSurface(t *testing.T) {
	vLo := r3.NewVec(1, -2, 0.5)
	vHi := r3.NewVec(4, 0, 1)
	wLo := r3.NewVec(-0.5, 2, 6)
	wHi := r3.NewVec(3, 4, 7.5)
	anchor := r3.NewVec(-2, 1.25, -3)

	coarseVol6, coarseMoment := bilinearFacetMoments(vLo, vHi, wLo, wHi, anchor, 1)
	mediumVol6, mediumMoment := bilinearFacetMoments(vLo, vHi, wLo, wHi, anchor, 256)
	fineVol6, fineMoment := bilinearFacetMoments(vLo, vHi, wLo, wHi, anchor, 512)
	wantVolumeCorrection, _ := cellTwistVolume(vLo, vHi, wLo, wHi).Float64()
	wantMomentCorrection := cellTwistMoment(vLo, vHi, wLo, wHi, anchor)

	refinedVol6 := (4*fineVol6 - mediumVol6) / 3
	require.InDelta(t, 6*wantVolumeCorrection, refinedVol6-coarseVol6, 1e-6)
	for axis, refined := range []float64{
		(4*fineMoment.X - mediumMoment.X) / 3,
		(4*fineMoment.Y - mediumMoment.Y) / 3,
		(4*fineMoment.Z - mediumMoment.Z) / 3,
	} {
		want, _ := wantMomentCorrection[axis].Float64()
		require.InDelta(t, want, refined-[]float64{coarseMoment.X, coarseMoment.Y, coarseMoment.Z}[axis], 1e-5,
			"the exact first-moment correction must match the independently refined bilinear surface on axis %d", axis)
	}
}

func TestComputeLoftChordedAllowReversesSignedCorrections(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(1, -2, 0.5), r3.NewVec(4, 0, 1),
		r3.NewVec(-0.5, 2, 6), r3.NewVec(3, 4, 7.5),
	}
	pairs := []loftLoopPair{{
		v:              make([]Point2, 2),
		w:              make([]Point2, 2),
		arcUpperV:      []float64{4, 0},
		arcUpperW:      []float64{4, 0},
		matchedDelta:   []float64{0.01, 0},
		tangentEnergyV: []float64{math.Inf(1), math.Inf(1)},
		tangentEnergyW: []float64{math.Inf(1), math.Inf(1)},
	}}
	vIdx, wIdx := [][]int{{0, 1}}, [][]int{{2, 3}}
	anchor := r3.NewVec(-2, 1.25, -3)

	forward, err := computeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.01, 0, 12, false)
	require.NoError(t, err)
	reversed, err := computeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.01, 0, 12, true)
	require.NoError(t, err)

	require.Zero(t, new(big.Rat).Add(forward.twistVolumeCorrection, reversed.twistVolumeCorrection).Sign())
	require.Equal(t, forward.twistVolumeUpper, reversed.twistVolumeUpper,
		"shell orientation must not change the unsigned occupied-volume measure")
	for axis := range forward.twistMomentCorrection {
		require.Zero(t,
			new(big.Rat).Add(forward.twistMomentCorrection[axis], reversed.twistMomentCorrection[axis]).Sign(),
			"reversing the shell must reverse the signed first-moment correction on axis %d", axis)
	}
	require.Equal(t, forward.areaCorrection, reversed.areaCorrection,
		"shell orientation must not change the unsigned area reading")
}

func bilinearAreaMidpoint(vLo, vHi, wLo, wHi r3.Vec, divisions int) float64 {
	a := vHi.Sub(vLo)
	b := wLo.Sub(vLo)
	twist := vLo.Sub(vHi).Sub(wLo).Add(wHi)
	h := 1 / float64(divisions)
	sum := 0.0
	for i := range divisions {
		s := (float64(i) + 0.5) * h
		for j := range divisions {
			r := (float64(j) + 0.5) * h
			n := a.Add(twist.Scale(r)).Cross(b.Add(twist.Scale(s)))
			sum += n.Len()
		}
	}
	return sum * h * h
}

func bilinearFacetMoments(vLo, vHi, wLo, wHi, anchor r3.Vec, divisions int) (float64, r3.Vec) {
	a := vHi.Sub(vLo)
	b := wLo.Sub(vLo)
	twist := vLo.Sub(vHi).Sub(wLo).Add(wHi)
	at := func(i, j int) r3.Vec {
		s := float64(i) / float64(divisions)
		r := float64(j) / float64(divisions)
		return vLo.Add(a.Scale(s)).Add(b.Scale(r)).Add(twist.Scale(s * r))
	}

	vol6 := 0.0
	moment := r3.Vec{}
	add := func(p0, p1, p2 r3.Vec) {
		q0, q1, q2 := p0.Sub(anchor), p1.Sub(anchor), p2.Sub(anchor)
		det := q0.Dot(q1.Cross(q2))
		vol6 += det
		moment = moment.Add(q0.Add(q1).Add(q2).Scale(det))
	}
	for i := range divisions {
		for j := range divisions {
			loLo := at(i, j)
			hiLo := at(i+1, j)
			loHi := at(i, j+1)
			hiHi := at(i+1, j+1)
			add(loLo, hiLo, hiHi)
			add(loLo, hiHi, loHi)
		}
	}
	return vol6, moment
}
