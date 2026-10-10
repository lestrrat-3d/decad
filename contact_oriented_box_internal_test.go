package decad

import (
	"math/big"
	"testing"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// orientedVertexFaceProjection holds the vertex-to-face foot in Dyadic
// numerators; this test holds it to the big.Rat evaluation of the same
// projection, kept below as orientedVertexFaceFootRational: every foot and
// squared distance must be the identical rational, and both must refuse the
// same vertices.

func orientedVertexFaceFootRational(vertex proofarith.DyV3, box orientedSourceBox, axis, side int) ([3]*big.Rat, *big.Rat) {
	i, j := (axis+1)%3, (axis+2)%3
	face := box.Corner[0]
	if side == 1 {
		face = proofarith.DvAdd(face, box.Edge[axis])
	}
	a, b := box.Edge[i], box.Edge[j]
	normal := proofarith.DvCross(a, b)
	normSquared := proofarith.DvDot(normal, normal).Rat()
	if normSquared.Sign() == 0 {
		return [3]*big.Rat{}, nil
	}
	w := proofarith.DvSub(vertex, face)
	distanceNumerator := proofarith.DvDot(w, normal).Rat()
	point := [3]*big.Rat{}
	for k := range 3 {
		point[k] = new(big.Rat).Sub(w[k].Rat(),
			new(big.Rat).Quo(new(big.Rat).Mul(normal[k].Rat(), distanceNumerator), normSquared))
	}
	dot := func(u, v [3]*big.Rat) *big.Rat {
		result := new(big.Rat)
		for k := range 3 {
			result.Add(result, new(big.Rat).Mul(u[k], v[k]))
		}
		return result
	}
	ar, br := [3]*big.Rat{a[0].Rat(), a[1].Rat(), a[2].Rat()},
		[3]*big.Rat{b[0].Rat(), b[1].Rat(), b[2].Rat()}
	aa, bb, ab := dot(ar, ar), dot(br, br), dot(ar, br)
	det := new(big.Rat).Sub(new(big.Rat).Mul(aa, bb), new(big.Rat).Mul(ab, ab))
	if det.Sign() <= 0 {
		return [3]*big.Rat{}, nil
	}
	pa, pb := dot(point, ar), dot(point, br)
	u := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(pa, bb),
		new(big.Rat).Mul(pb, ab)), det)
	v := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(pb, aa),
		new(big.Rat).Mul(pa, ab)), det)
	if u.Sign() < 0 || v.Sign() < 0 || u.Cmp(big.NewRat(1, 1)) > 0 || v.Cmp(big.NewRat(1, 1)) > 0 {
		return [3]*big.Rat{}, nil
	}
	var foot [3]*big.Rat
	for k := range 3 {
		foot[k] = new(big.Rat).Add(face[k].Rat(), point[k])
	}
	return foot, new(big.Rat).Quo(new(big.Rat).Mul(distanceNumerator, distanceNumerator), normSquared)
}

func TestOrientedVertexFaceFootMatchesRationalForm(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, -6, -4, 6, 4, 10)
	var boxes []orientedSourceBox
	for _, turn := range []struct {
		axis    r3.Vec
		degrees float64
	}{{r3.Vec{Z: 1}, 0}, {r3.Vec{X: 1, Y: 1}, 30}, {r3.Vec{X: 1, Y: 2, Z: 3}, 50}, {r3.Vec{X: 3, Y: -1, Z: 2}, 140}} {
		pose, err := r3.RotationAround(r3.Vec{X: 1, Y: -2, Z: 3}, turn.axis, units.Degrees(turn.degrees))
		require.NoError(t, err)
		oriented, ok := sourceOrientedBoxAtPose(box, pose)
		require.True(t, ok)
		boxes = append(boxes, oriented)
	}
	inside, outside := 0, 0
	for _, target := range boxes {
		for _, from := range boxes {
			vertices := pairbox.OrientedWitnessSamples(from.OrientedBox)
			for _, offset := range []float64{0, 3.25, -7.5} {
				for _, sample := range pairbox.OrientedWitnessSamples(from.OrientedBox)[:8] {
					shifted := proofarith.DvAdd(sample, proofarith.DyVec(r3.Vec{X: offset, Y: offset / 2, Z: -offset}))
					vertices = append(vertices, shifted)
				}
			}
			for _, vertex := range vertices {
				for axis := range 3 {
					for side := range 2 {
						wantFoot, wantDistance := orientedVertexFaceFootRational(vertex, target, axis, side)
						gotFoot, gotDistance := pairbox.OrientedVertexFaceFoot(vertex, target.OrientedBox, axis, side)
						require.Equal(t, wantDistance == nil, gotDistance == nil)
						require.Equal(t, wantDistance == nil,
							pairbox.OrientedVertexFaceDistanceSquared(vertex, target.OrientedBox, axis, side) == nil)
						if wantDistance == nil {
							outside++
							continue
						}
						inside++
						require.Zero(t, wantDistance.Cmp(gotDistance))
						require.Zero(t, wantDistance.Cmp(pairbox.OrientedVertexFaceDistanceSquared(
							vertex, target.OrientedBox, axis, side)))
						for k := range 3 {
							require.Zero(t, wantFoot[k].Cmp(gotFoot[k]))
						}
					}
				}
			}
		}
	}
	require.Positive(t, inside, "premise: some feet land in their face")
	require.Positive(t, outside, "premise: some feet miss their face")
}
