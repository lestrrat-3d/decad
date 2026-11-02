package tessellation

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// blindContactInt represents a signed integer with a fixed 2048-bit magnitude.
type blindContactInt struct {
	limbs    [32]uint64
	negative bool
	used     uint8
	known    bool
}

// blindContactCoordinate lifts an admitted binary64 coordinate exactly after
// scaling by 2^116. Its normal exponent is in [-64,31], so the result needs at
// most 148 bits. Both zero signs have the same integer representation.
func blindContactCoordinate(x float64, out *blindContactInt) bool {
	b := math.Float64bits(x)
	exponent := int((b >> 52) & 0x7ff)
	fraction := b & ((uint64(1) << 52) - 1)
	if exponent == 0 {
		if out != nil {
			*out = blindContactInt{}
			out.known = true
		}
		return fraction == 0
	}
	e := exponent - 1023
	if exponent == 0x7ff || e < -64 || e > 31 {
		return false
	}
	if out == nil {
		return true
	}
	*out = blindContactInt{}
	m := fraction | uint64(1)<<52
	shift := uint(e + 64)
	word, bit := shift/64, shift%64
	out.limbs[word] = m << bit
	out.used = uint8(word + 1)
	if bit != 0 {
		out.limbs[word+1] = m >> (64 - bit)
		if out.limbs[word+1] != 0 {
			out.used++
		}
	}
	out.negative = b>>63 != 0
	out.known = true
	return true
}

type blindContactBox struct {
	min, max [3]float64
}

func contactAxis(p r3.Vec, axis int) float64 {
	if axis == 0 {
		return p.X
	}
	if axis == 1 {
		return p.Y
	}
	return p.Z
}

// contactVertexSeparated proves that two facets meet only at their shared
// index when one has both other corners strictly on one side of an axis plane
// and the other remains in the opposite closed half-space.
func contactVertexSeparated(ctx context.Context, budget *proofbound.ReplayBudget,
	vertices []r3.Vec, x, y [3]int) (bool, error) {
	if err := budget.SpendWork(ctx, 160); err != nil {
		return false, err
	}
	shared, count := -1, 0
	for _, i := range x {
		for _, j := range y {
			if i == j {
				shared, count = i, count+1
			}
		}
	}
	if count != 1 {
		return false, nil
	}
	for axis := range 3 {
		p := contactAxis(vertices[shared], axis)
		xmin, xmax := math.Inf(1), math.Inf(-1)
		ymin, ymax := math.Inf(1), math.Inf(-1)
		for _, index := range x {
			if index != shared {
				v := contactAxis(vertices[index], axis)
				xmin, xmax = min(xmin, v), max(xmax, v)
			}
		}
		for _, index := range y {
			if index != shared {
				v := contactAxis(vertices[index], axis)
				ymin, ymax = min(ymin, v), max(ymax, v)
			}
		}
		if xmin > p && ymax <= p || xmax < p && ymin >= p ||
			ymin > p && xmax <= p || ymax < p && xmin >= p {
			return true, nil
		}
	}
	return false, nil
}

type blindContactScratch struct {
	points    [2][3]blindContactVector
	distances [2][3]blindContactInt
	endpoints [2][2][4]blindContactInt
	products  [2]blindContactInt
	orient    [8]blindContactInt
	dot       [3]blindContactInt
	projected [2][2]blindContactInt
	vectors   [3]blindContactVector
	zero      blindContactInt
}

func blindContactStorageBytes(vertices, facets uint64) uint64 {
	return 2*(facets*uint64(unsafe.Sizeof(blindContactBox{}))+
		(vertices+facets)*uint64(unsafe.Sizeof(blindContactVector{}))+
		2*uint64(unsafe.Sizeof(blindContactScratch{}))) + 4*128 + 65536
}

// The worker's fixed arena is initialized only after this work and the stack
// envelope have been reserved by its caller. The extra work covers fixed
// frame setup and zeroing beyond the arena itself.
func blindContactStackWork() uint64 {
	return (uint64(unsafe.Sizeof(blindContactScratch{}))+7)/8 + 2048
}

func reserveBlindContactStorage(ctx context.Context, budget *proofbound.ReplayBudget, vertices, facets uint64) error {
	if vertices > 65536 || facets > 4096 {
		return decaderr.ErrUnsupported
	}
	return budget.ReserveBundle(ctx, 0, vertices+facets+1, blindContactStorageBytes(vertices, facets))
}

func reserveBlindContactScan(ctx context.Context, budget *proofbound.ReplayBudget, facets uint64) error {
	if facets > 4096 || facets > 1 && facets*(facets-1)/2 > 1000000 {
		return decaderr.ErrUnsupported
	}
	pairs := facets * (facets - min(facets, 1)) / 2
	return budget.ReserveBundle(ctx, 6*pairs, facets, 0)
}

func reserveBlindContactPlan(ctx context.Context, budget *proofbound.ReplayBudget, vertices, facets uint64) error {
	if vertices > 65536 || facets > 4096 || facets > 1 && facets*(facets-1)/2 > 1000000 {
		return decaderr.ErrUnsupported
	}
	pairs := facets * (facets - min(facets, 1)) / 2
	return budget.ReserveBundle(ctx, 6*pairs+blindContactStackWork(), vertices+2*facets+1, blindContactStorageBytes(vertices, facets))
}

// contactNormal rejects collapsed facets before any pair is classified.
func (a blindContactArithmetic) contactNormal(p, q, r, normal *blindContactVector, scratch *blindContactScratch) error {
	if err := a.differenceContactInto(&scratch.vectors[0], q, p); err != nil {
		return err
	}
	if err := a.differenceContactInto(&scratch.vectors[1], r, p); err != nil {
		return err
	}
	if err := a.crossContactInto(normal, &scratch.vectors[0], &scratch.vectors[1], &scratch.products); err != nil {
		return err
	}
	nonzero := false
	for i := range normal {
		if err := a.requireBits(&normal[i], 300); err != nil {
			return err
		}
		sign, err := a.compare(&normal[i], &scratch.zero)
		if err != nil {
			return err
		}
		nonzero = nonzero || sign != 0
	}
	if !nonzero {
		return decaderr.ErrUnsupported
	}
	return nil
}

// contactPair checks the indexed intersection of two noncollapsed facets.
// The caller has authenticated the indices and computed their normals.
func (a blindContactArithmetic) contactPair(vertices []blindContactVector, floats []r3.Vec, normals []blindContactVector,
	x, y [3]int, xi, yi int, scratch *blindContactScratch) (bool, error) {
	// The pair visits at most nine index pairs, six corners, six plane
	// distances, three normal components and two clips. Charge 512 for those
	// fixed branches and indices; copies and predicates charge separately.
	if err := a.budget.SpendWork(a.ctx, 512); err != nil {
		return false, err
	}
	scratch.zero.known = true
	scratch.orient[7].known = true
	var shared [2]int
	count := 0
	for _, i := range x {
		for _, j := range y {
			if i == j {
				if count == len(shared) {
					return false, nil
				}
				shared[count] = i
				count++
			}
		}
	}
	if count == 2 {
		// The shared indices are distinct corners of two noncollapsed facets.
		// A nonzero distance of the remaining corner proves distinct planes,
		// whose intersection is exactly the shared edge.
		third := -1
		for _, index := range y {
			if index != shared[0] && index != shared[1] {
				third = index
			}
		}
		if third < 0 {
			return false, decaderr.ErrUnsupported
		}
		if err := a.differenceContactInto(&scratch.vectors[0], &vertices[third], &vertices[x[0]]); err != nil {
			return false, err
		}
		distance := &scratch.distances[0][0]
		if err := a.dotContactInto(distance, &scratch.vectors[0], &normals[xi], &scratch.dot); err != nil {
			return false, err
		}
		if err := a.requireBits(distance, 450); err != nil {
			return false, err
		}
		sign, err := a.compare(distance, &scratch.zero)
		if err != nil {
			return false, err
		}
		if sign != 0 {
			return true, nil
		}
		return a.coplanarPair(vertices, floats, normals, x, y, shared, count, xi, scratch)
	}
	triangles := [2][3]int{x, y}
	distances := &scratch.distances
	coplanar := true
	for i := range triangles {
		other := 1 - i
		normal := &normals[yi]
		if i == 1 {
			normal = &normals[xi]
		}
		positive, negative := 0, 0
		for j := range triangles[i] {
			if count == 1 && triangles[i][j] == shared[0] {
				if err := a.budget.SpendWork(a.ctx, 33); err != nil {
					return false, err
				}
				distances[i][j] = blindContactInt{known: true}
				continue
			}
			if err := a.differenceContactInto(&scratch.vectors[0], &vertices[triangles[i][j]], &vertices[triangles[other][0]]); err != nil {
				return false, err
			}
			if err := a.dotContactInto(&distances[i][j], &scratch.vectors[0], normal, &scratch.dot); err != nil {
				return false, err
			}
			if err := a.requireBits(&distances[i][j], 450); err != nil {
				return false, err
			}
			s, err := a.compare(&distances[i][j], &scratch.zero)
			if err != nil {
				return false, err
			}
			if s > 0 {
				positive++
			}
			if s < 0 {
				negative++
			}
			coplanar = coplanar && s == 0
		}
		// Convexity keeps a facet with three strict same-side signs away
		// from the other plane. With one indexed shared corner, two strict
		// same-side signs confine its contact to that corner.
		if count == 0 && (positive == 3 || negative == 3) ||
			count == 1 && (positive == 2 || negative == 2) {
			return true, nil
		}
	}
	if coplanar {
		return a.coplanarPair(vertices, floats, normals, x, y, shared, count, xi, scratch)
	}
	direction := &scratch.vectors[2]
	if err := a.crossContactInto(direction, &normals[xi], &normals[yi], &scratch.products); err != nil {
		return false, err
	}
	for i := range direction {
		if err := a.requireBits(&direction[i], 600); err != nil {
			return false, err
		}
	}
	integerWords := (uint64(unsafe.Sizeof(blindContactInt{})) + 7) / 8
	if err := a.budget.SpendWork(a.ctx, 18*integerWords); err != nil {
		return false, err
	}
	p := &scratch.points
	for i, tri := range triangles {
		for j, index := range tri {
			p[i][j] = vertices[index]
		}
	}
	endpoints := &scratch.endpoints
	var counts [2]int
	for i := range p {
		var err error
		counts[i], err = a.clipContactTriangle(&p[i], &distances[i], &endpoints[i], &scratch.products, &scratch.zero)
		if err != nil {
			return false, err
		}
	}
	return a.noncoplanarContactAllowed(&endpoints[0], &endpoints[1], counts[0], counts[1], count, direction, scratch)
}

func (a blindContactArithmetic) coplanarPair(vertices []blindContactVector, floats []r3.Vec, normals []blindContactVector,
	x, y [3]int, shared [2]int, count, xi int, scratch *blindContactScratch) (bool, error) {
	for i := range 3 {
		sign, err := a.compare(&normals[xi][i], &scratch.zero)
		if err != nil {
			return false, err
		}
		if sign != 0 {
			return a.coplanarContactAllowed(vertices, floats, x, y, shared, count, (i+1)%3, (i+2)%3, &scratch.orient)
		}
	}
	return false, decaderr.ErrUnsupported
}

// ReplayRequireEmbeddedBoundary proves that indexed facets meet only along
// their recorded shared vertices and edges.
func ReplayRequireEmbeddedBoundary(ctx context.Context, vertices []r3.Vec, triangles [][3]int,
	budget *proofbound.ReplayBudget) error {
	if budget == nil || len(vertices) > 65536 || len(triangles) > 4096 {
		return decaderr.ErrUnsupported
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	n := uint64(len(triangles))
	if n > 1 && n*(n-1)/2 > 1000000 {
		return decaderr.ErrUnsupported
	}
	for _, p := range vertices {
		for _, x := range [...]float64{p.X, p.Y, p.Z} {
			if err := budget.SpendWork(ctx, 4); err != nil {
				return err
			}
			if !blindContactCoordinate(x, nil) {
				return decaderr.ErrUnsupported
			}
		}
	}
	if n == 0 {
		return decaderr.ErrUnsupported
	}
	// Charge both comparisons on every axis before examining any pair.
	// The worker owns one reusable fixed arena. Reserve its stack envelope before
	// entering the worker, so its frame cannot grow before the reservation.
	if err := reserveBlindContactPlan(ctx, budget, uint64(len(vertices)), n); err != nil {
		return err
	}
	return replayEmbeddedBoundary(ctx, vertices, triangles, budget)
}

func replayEmbeddedBoundary(ctx context.Context, vertices []r3.Vec, triangles [][3]int,
	budget *proofbound.ReplayBudget) error {
	var scratch blindContactScratch
	scratch.zero.known = true
	boxes := make([]blindContactBox, len(triangles))
	exact := make([]blindContactVector, len(vertices))
	normals := make([]blindContactVector, len(triangles))
	for i, p := range vertices {
		for j, x := range [...]float64{p.X, p.Y, p.Z} {
			if err := budget.SpendWork(ctx, 37); err != nil {
				return err
			}
			blindContactCoordinate(x, &exact[i][j])
		}
	}
	a := blindContactArithmetic{ctx, budget}
	for i, tri := range triangles {
		if err := budget.SpendWork(ctx, 24); err != nil {
			return err
		}
		for j, index := range tri {
			if index < 0 || index >= len(vertices) || j > 0 && (index == tri[0] || j == 2 && index == tri[1]) {
				return decaderr.ErrUnsupported
			}
			p := vertices[index]
			for k, x := range [...]float64{p.X, p.Y, p.Z} {
				if j == 0 || x < boxes[i].min[k] {
					boxes[i].min[k] = x
				}
				if j == 0 || x > boxes[i].max[k] {
					boxes[i].max[k] = x
				}
			}
		}
		if err := a.contactNormal(&exact[tri[0]], &exact[tri[1]], &exact[tri[2]], &normals[i], &scratch); err != nil {
			return err
		}
	}
	for i := range boxes {
		for j := i + 1; j < len(boxes); j++ {
			if err := ctx.Err(); err != nil {
				return err
			}
			disjoint := false
			for k := range 3 {
				if boxes[i].max[k] < boxes[j].min[k] || boxes[j].max[k] < boxes[i].min[k] {
					disjoint = true
					break
				}
			}
			if !disjoint {
				separated, err := contactVertexSeparated(ctx, budget, vertices, triangles[i], triangles[j])
				if err != nil {
					return err
				}
				if separated {
					continue
				}
				ok, err := a.contactPair(exact, vertices, normals, triangles[i], triangles[j], i, j, &scratch)
				if err != nil {
					return err
				}
				if !ok {
					return decaderr.ErrUnsupported
				}
			}
		}
	}
	return ctx.Err()
}
