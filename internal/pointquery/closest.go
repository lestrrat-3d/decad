package pointquery

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Closest is the nearest point of a held mesh to a query point, with the
// distance reading that goes with it.
type Closest struct {
	// Point is the nearest mesh point, rounded to the nearest floats, or the
	// query point itself when it lies in or on the solid.
	Point r3.Vec
	// Distance bounds the true distance from the query point to the body or
	// face, as BodyDistance and FaceDistance read it.
	Distance measurement.Measurement
	// OffSurface bounds, in millimetres, how far Point lies from the true
	// boundary: the mesh's displacement bound plus the rounding of Point.
	OffSurface float64
}

// segmentNearest returns the squared distance from p to the segment ab and the
// segment's point nearest p.
func segmentNearest(p, a, b ratVec) (*big.Rat, ratVec) {
	axis := subtract(b, a)
	lengthSquared := dot(axis, axis)
	fromStart := subtract(p, a)
	at := func(q ratVec) (*big.Rat, ratVec) {
		d := subtract(p, q)
		return dot(d, d), q
	}
	if lengthSquared.Sign() == 0 {
		return at(a)
	}
	along := dot(fromStart, axis)
	if along.Sign() <= 0 {
		return at(a)
	}
	if along.Cmp(lengthSquared) >= 0 {
		return at(b)
	}
	t := new(big.Rat).Quo(along, lengthSquared)
	foot := ratVec{}
	for i := range foot {
		foot[i] = new(big.Rat).Add(a[i], new(big.Rat).Mul(t, axis[i]))
	}
	return at(foot)
}

// triangleNearest returns the squared distance from p to the triangle abc and
// the triangle's point nearest p.
func triangleNearest(p, a, b, c ratVec) (*big.Rat, ratVec, error) {
	normal := cross(subtract(b, a), subtract(c, a))
	normalSquared := dot(normal, normal)
	if normalSquared.Sign() == 0 {
		return nil, ratVec{}, fmt.Errorf(`%w: point query mesh contains a degenerate triangle`, decaderr.ErrUnsupported)
	}
	triangle := [3]ratVec{a, b, c}
	inside := true
	for i := range triangle {
		start, end := triangle[i], triangle[(i+1)%3]
		if dot(cross(subtract(end, start), subtract(p, start)), normal).Sign() < 0 {
			inside = false
			break
		}
	}
	if inside {
		offset := dot(subtract(p, a), normal)
		scale := new(big.Rat).Quo(offset, normalSquared)
		foot := ratVec{}
		for i := range foot {
			foot[i] = new(big.Rat).Sub(p[i], new(big.Rat).Mul(scale, normal[i]))
		}
		return new(big.Rat).Mul(scale, offset), foot, nil
	}
	var best *big.Rat
	var bestPoint ratVec
	for i := range triangle {
		d, q := segmentNearest(p, triangle[i], triangle[(i+1)%3])
		if best == nil || d.Cmp(best) < 0 {
			best, bestPoint = d, q
		}
	}
	return best, bestPoint, nil
}

// MeshClosest returns the exact squared distance from p to the held triangle
// set and the set's nearest point, as exact rationals.
func MeshClosest(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (*big.Rat, [3]*big.Rat, error) {
	if len(tris) == 0 {
		return nil, [3]*big.Rat{}, fmt.Errorf(`%w: point query mesh has no triangles`, decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	point := exactVec(p)
	if err := checkFiniteVertices(budget, verts); err != nil {
		return nil, [3]*big.Rat{}, err
	}
	exact := newLazyVertices(verts)
	var best *big.Rat
	var bestUpper float64
	var bestPoint ratVec
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, [3]*big.Rat{}, err
		}
		for _, index := range tri {
			if index < 0 || index >= len(verts) {
				return nil, [3]*big.Rat{}, fmt.Errorf(`%w: point query mesh has an invalid triangle index`, decaderr.ErrUnsupported)
			}
		}
		if best != nil && boxDistanceSquaredLower(p, verts[tri[0]], verts[tri[1]], verts[tri[2]]) >= bestUpper {
			continue
		}
		a, b, c := exact.at(tri[0]), exact.at(tri[1]), exact.at(tri[2])
		if best != nil && boxDistanceSquared(point, a, b, c).Cmp(best) >= 0 {
			continue
		}
		d, q, err := triangleNearest(point, a, b, c)
		if err != nil {
			return nil, [3]*big.Rat{}, err
		}
		if best == nil || d.Cmp(best) < 0 {
			best, bestPoint = d, q
			bestUpper = proofbound.RatFloatUp(best)
		}
	}
	return best, bestPoint, nil
}

// roundPoint rounds an exact point to the nearest floats and returns the
// distance, rounded up, between the two.
func roundPoint(q [3]*big.Rat) (r3.Vec, float64, error) {
	var v [3]float64
	gap := new(big.Rat)
	for i := range q {
		v[i] = sweeppath.RatFloatNearest(q[i])
		if !measurement.FiniteValues(v[i]) {
			return r3.Vec{}, 0, fmt.Errorf(`%w: the closest point is not representable`, decaderr.ErrUnsupported)
		}
		d := new(big.Rat).Sub(q[i], proofarith.FloatRat(v[i]))
		gap.Add(gap, d.Mul(d, d))
	}
	return r3.Vec{X: v[0], Y: v[1], Z: v[2]}, proofbound.RatSqrtUp(gap), nil
}

// BodyClosest reads the nearest point of the solid a VerifyAll mesh stands for.
// A point in or on the solid is its own nearest point.
func BodyClosest(ctx context.Context, p r3.Vec, mesh *topo.Mesh) (Closest, error) {
	if !mesh.BoundaryOK || proofbound.IsNonFinite(mesh.Bound) || mesh.Bound < 0 {
		return Closest{}, fmt.Errorf(`%w: closest point needs a verified finite mesh boundary bound`, decaderr.ErrUnsupported)
	}
	distanceSquared, nearest, err := MeshClosest(ctx, p, mesh.Vertices, mesh.Triangles)
	if err != nil {
		return Closest{}, err
	}
	distance, location, err := bodyDistanceFrom(ctx, p, mesh, distanceSquared)
	if err != nil {
		return Closest{}, err
	}
	if location == Inside || location == OnBoundary {
		return Closest{Point: p, Distance: distance}, nil
	}
	point, round, err := roundPoint(nearest)
	if err != nil {
		return Closest{}, err
	}
	return Closest{Point: point, Distance: distance, OffSurface: sumUp(mesh.Bound, round)}, nil
}

// FaceClosest reads the nearest point of the trimmed patch of face.
func FaceClosest(ctx context.Context, p r3.Vec, mesh *topo.Mesh, face *topo.Face) (Closest, error) {
	distance, err := FaceDistance(ctx, p, mesh, face)
	if err != nil {
		return Closest{}, err
	}
	triangles := make([][3]int, 0)
	for i, source := range mesh.Source {
		if source == face {
			triangles = append(triangles, mesh.Triangles[i])
		}
	}
	_, nearest, err := MeshClosest(ctx, p, mesh.Vertices, triangles)
	if err != nil {
		return Closest{}, err
	}
	point, round, err := roundPoint(nearest)
	if err != nil {
		return Closest{}, err
	}
	return Closest{Point: point, Distance: distance, OffSurface: sumUp(mesh.FaceBound[face], round)}, nil
}

// sumUp returns a float no less than the exact sum a + b.
func sumUp(a, b float64) float64 {
	return proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(a), proofarith.FloatRat(b)))
}
