package pair

import (
	"errors"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// supportSetRebuilt is one plane's lifted set built the way PlanarSupportSets
// must reproduce: both solids' derived data and adjacency built afresh for
// the plane alone.
func supportSetRebuilt(a, b *PlanarSolid, plane SupportPlane, band proof.Dyadic, overlap bool,
	poll func() error) ([]PatchPoint, error) {
	if len(a.Faces) != len(a.Tris) || len(b.Faces) != len(b.Tris) || band.Sign() <= 0 {
		return nil, nil
	}
	hostSolid, guestSolid := b, a
	if plane.HostIsA {
		hostSolid, guestSolid = a, b
	}
	host := newPatchSide(hostSolid, false, plane.HostIsA)
	guest := newPatchSide(guestSolid, false, !plane.HostIsA)
	if _, ok := host.faceTris[plane.Face]; !ok || !host.isFlat(plane.Face) {
		return nil, nil
	}
	n, o := host.faceNormal(plane.Face), host.faceOrigin(plane.Face)
	heights := make([]proof.Dyadic, len(guestSolid.Verts))
	lowest := proof.DyZero()
	for v, at := range guestSolid.Verts {
		if err := poll(); err != nil {
			return nil, err
		}
		heights[v] = proof.DvDot(n, proof.DvSub(at, o))
		if v == 0 || proof.DyCmp(heights[v], lowest) < 0 {
			lowest = heights[v]
		}
	}
	if (lowest.Sign() < 0) != overlap {
		return nil, nil
	}
	frame := NewPlaneFrame(n, o)
	outer, holes, ok := host.frameLoops(plane.Face, frame, Point3{})
	if !ok {
		return nil, nil
	}
	region := append([][]Point2{outer}, holes...)
	norm := proof.DvDot(n, n)
	limit := proof.DyMul(proof.DyMul(band, band), norm)
	hostFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{plane.Face}}
	var points []PatchPoint
	for v, h := range heights {
		if err := poll(); err != nil {
			return nil, err
		}
		if lowest.Sign() <= 0 && proof.DyCmp(h, lowest) == 0 {
			continue
		}
		if h.Sign() > 0 && proof.DyCmp(proof.DyMul(h, h), limit) > 0 {
			continue
		}
		vertex := ratPoint3(guestSolid.Verts[v])
		lift := new(big.Rat).Quo(h.Rat(), norm.Rat())
		var foot Point3
		for axis := range 3 {
			foot[axis] = new(big.Rat).Sub(vertex[axis], new(big.Rat).Mul(lift, n[axis].Rat()))
		}
		if locate(frame.Project(foot), region) <= 0 {
			continue
		}
		var separation ScalarReading
		if h.Sign() != 0 {
			reading, ok := canonicalSqrt(frac{num: proof.DyMul(h, h), den: norm})
			if !ok {
				return nil, nil
			}
			separation = reading
			if h.Sign() < 0 {
				separation.ValueMM = -separation.ValueMM
			}
		}
		feature := PatchFeature{Kind: FeatureVertex, Faces: guest.faceIDs(guest.vertTris[v])}
		point := PatchPoint{OnA: foot, OnB: vertex, A: hostFeature, B: feature, Normal: n, Separation: separation}
		if !plane.HostIsA {
			point.OnA, point.OnB, point.A, point.B, point.Normal = vertex, foot, feature, hostFeature, dvNeg(n)
		}
		points = append(points, point)
	}
	return points, nil
}

// supportSetsRebuilt concatenates supportSetRebuilt over the planes in order.
func supportSetsRebuilt(a, b *PlanarSolid, planes []SupportPlane, band proof.Dyadic, overlap bool,
	poll func() error) ([]PatchPoint, error) {
	var out []PatchPoint
	for _, plane := range planes {
		points, err := supportSetRebuilt(a, b, plane, band, overlap, poll)
		if err != nil {
			return nil, err
		}
		out = append(out, points...)
	}
	return out, nil
}

// boxQuads are a unit cube's faces as outward-wound corner quads, corner i at
// (i&1, i>>1&1, i>>2&1): −x, +x, −y, +y, −z, +z.
var boxQuads = [6][4]int{{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 2, 3, 1}, {4, 5, 7, 6}}

// shearedBox is the box [lo, lo+size] sheared by x += sx·z, y += sy·z, which
// keeps each face flat and the volume positive, with the faces numbered from
// first in the order of boxQuads. When split is set its +z face is fanned from
// a center vertex into four triangles.
func shearedBox(lo, size [3]proof.Dyadic, sx, sy proof.Dyadic, first int, split bool) PlanarSolid {
	var s PlanarSolid
	for i := range 8 {
		var p proof.DyV3
		for axis := range 3 {
			p[axis] = lo[axis]
			if i>>axis&1 == 1 {
				p[axis] = proof.DyAdd(lo[axis], size[axis])
			}
		}
		p[0] = proof.DyAdd(p[0], proof.DyMul(sx, p[2]))
		p[1] = proof.DyAdd(p[1], proof.DyMul(sy, p[2]))
		s.Verts = append(s.Verts, p)
	}
	for f, q := range boxQuads {
		if split && f == 5 {
			var center proof.DyV3
			for axis := range 3 {
				sum := proof.DyZero()
				for _, v := range q {
					sum = proof.DyAdd(sum, s.Verts[v][axis])
				}
				center[axis] = proof.DyShift(sum, -2)
			}
			c := len(s.Verts)
			s.Verts = append(s.Verts, center)
			for i := range 4 {
				s.Tris = append(s.Tris, [3]int{q[i], q[(i+1)%4], c})
				s.Faces = append(s.Faces, first+f)
			}
			continue
		}
		s.Tris = append(s.Tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
		s.Faces = append(s.Faces, first+f, first+f)
	}
	return s
}

// TestPlanarSupportSetsMatchRebuilt holds PlanarSupportSets, which builds each
// solid's derived data and adjacency once for every plane and reuses
// ClassifyPlanar's, to the per-plane rebuild it replaces: the same points in
// the same order, the same polls, and the same error at the same poll.
// Boxes on a quarter grid, some sheared and some with a fanned top face,
// rest on, hover over and sink into each other, so planes hold empty, touch
// and overlap sets. Every face of both bodies is a plane, plus repeated faces
// and a face neither body has.
//
// Legs shown to fail: preparedFor returning the first prep whatever the
// solid, the classified-result call reads one body for both sides; the host
// and guest sides swapped in PlanarSupportSets, the polls and points differ
// on the first draw with A as host.
func TestPlanarSupportSetsMatchRebuilt(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(61, 67))
	quarter := func(lo, hi int) proof.Dyadic {
		return proof.DyShift(proof.DyInt(int64(lo+rng.IntN(hi-lo+1))), -2)
	}
	poll := func(count *int, stopAt int) func() error {
		return func() error {
			*count++
			if stopAt > 0 && *count == stopAt {
				return errStopPoll
			}
			return nil
		}
	}
	nonempty, empty, overlapping, stopped := 0, 0, 0, 0
	for trial := range 400 {
		size := [3]proof.Dyadic{quarter(4, 16), quarter(4, 16), quarter(4, 12)}
		floor := shearedBox([3]proof.Dyadic{proof.DyZero(), proof.DyZero(), proof.DyNeg(size[2])}, size,
			proof.DyZero(), proof.DyZero(), 0, trial%3 == 0)
		// The guest's bottom sits near z = 0: on it, a quarter above, or a
		// quarter or two below.
		lo := [3]proof.Dyadic{quarter(-4, 12), quarter(-4, 12), quarter(-2, 2)}
		sx, sy := proof.DyZero(), proof.DyZero()
		if trial%4 == 1 {
			sx, sy = quarter(-1, 1), quarter(-1, 1)
		}
		guest := shearedBox(lo, [3]proof.Dyadic{quarter(2, 8), quarter(2, 8), quarter(2, 8)}, sx, sy,
			3*(trial%2), trial%5 == 0)
		a, b := &guest, &floor
		if trial%2 == 1 {
			a, b = b, a
		}
		for _, s := range []*PlanarSolid{a, b} {
			ok, err := CheckPlanarSolid(s, noPollInternal)
			require.NoError(t, err)
			require.True(t, ok, "trial %d: premise: an audited solid", trial)
		}
		var planes []SupportPlane
		for _, host := range []struct {
			isA   bool
			solid *PlanarSolid
		}{{false, b}, {true, a}} {
			seen := map[int]struct{}{}
			for _, f := range host.solid.Faces {
				if _, ok := seen[f]; !ok {
					seen[f] = struct{}{}
					planes = append(planes, SupportPlane{HostIsA: host.isA, Face: f})
				}
			}
		}
		planes = append(planes, planes[rng.IntN(len(planes))], SupportPlane{Face: 99})
		rng.Shuffle(len(planes), func(i, j int) { planes[i], planes[j] = planes[j], planes[i] })
		band := quarter(1, 12)
		classified, err := ClassifyPlanar(a, b, noPollInternal)
		require.NoError(t, err)
		swapped, err := ClassifyPlanar(b, a, noPollInternal)
		require.NoError(t, err)
		for _, overlap := range []bool{false, true} {
			var wantPolls int
			want, err := supportSetsRebuilt(a, b, planes, band, overlap, poll(&wantPolls, 0))
			require.NoError(t, err)
			for _, result := range []PlanarResult{{}, classified, swapped} {
				var gotPolls int
				got, err := PlanarSupportSets(a, b, result, planes, band, overlap, poll(&gotPolls, 0))
				require.NoError(t, err)
				require.Equal(t, want, got, "trial %d overlap %v", trial, overlap)
				require.Equal(t, wantPolls, gotPolls, "trial %d overlap %v: polls", trial, overlap)
			}
			if wantPolls > 1 {
				stopAt := 1 + rng.IntN(wantPolls)
				var wantCount, gotCount int
				wantStop, wantErr := supportSetsRebuilt(a, b, planes, band, overlap, poll(&wantCount, stopAt))
				gotStop, gotErr := PlanarSupportSets(a, b, classified, planes, band, overlap, poll(&gotCount, stopAt))
				require.ErrorIs(t, wantErr, errStopPoll)
				require.ErrorIs(t, gotErr, errStopPoll)
				require.Equal(t, wantStop, gotStop)
				require.Equal(t, wantCount, gotCount)
				stopped++
			}
			switch {
			case want == nil:
				empty++
			case overlap:
				overlapping++
				nonempty++
			default:
				nonempty++
			}
		}
	}
	require.Positive(t, nonempty, "premise: some draws publish points")
	require.Positive(t, empty, "premise: some draws publish none")
	require.Positive(t, overlapping, "premise: some overlapping draws publish points")
	require.Positive(t, stopped, "premise: some draws stop at a poll")
}

var errStopPoll = errors.New("stop")
