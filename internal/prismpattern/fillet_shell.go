package prismpattern

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// FilletShell replaces the two root-wall junctions selected on
// each section with sampled line/circle tangent bands. It retains all other
// source wall facets and rebuilds both caps from the new common outline.
func FilletShell(ctx context.Context, rec *facetedrecord.PrismPatternJoinRecord,
	contacts []FilletContact, radius float64, ref provenance.Producer,
) (*facetedrecord.JoinedMesh, [2][]int, float64, error) {
	var outlines [2][]int
	// A contact may consume several original circular-wall stations. Keep
	// the final bracket facet and move its near endpoint to the circle foot.
	positionByVertex := make(map[int]int, 2*len(rec.Outlines[0]))
	for _, outline := range rec.Outlines {
		for position, vertex := range outline {
			positionByVertex[vertex] = position
		}
	}
	consumed := make(map[int]bool)
	dropPairs := make(map[[2]int]bool)
	farStations := make([]int, 0, len(contacts))
	for _, contact := range contacts {
		cut := contact.RingCut
		if len(cut.Positions) == 0 {
			return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet lost its ring bracket", decaderr.ErrUnsupported)
		}
		for i, position := range cut.Positions {
			if consumed[position] {
				return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring cuts overlap", decaderr.ErrUnsupported)
			}
			consumed[position] = true
			if i > 0 {
				pair := [2]int{cut.Positions[i-1], position}
				if pair[0] > pair[1] {
					pair[0], pair[1] = pair[1], pair[0]
				}
				dropPairs[pair] = true
			}
		}
		farStations = append(farStations, cut.Far)
	}
	for _, far := range farStations {
		if consumed[far] {
			return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring brackets overlap", decaderr.ErrUnsupported)
		}
	}
	dropCounts := make(map[[2]int]int, len(dropPairs))
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), rec.Mesh.Verts...),
		Groups: append([]facetedrecord.Group(nil), rec.Mesh.Groups...)}
	for i, tri := range rec.Mesh.Tris {
		group := rec.Mesh.Src[i]
		if group == rec.Groups[1] || group == rec.Groups[2] {
			continue
		}
		if group == rec.Groups[0] {
			pair := [2]int{-1, -1}
			for _, vertex := range tri {
				position, ok := positionByVertex[vertex]
				if !ok {
					return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring facet lost its station", decaderr.ErrUnsupported)
				}
				if position == pair[0] || position == pair[1] {
					continue
				}
				if pair[0] == -1 {
					pair[0] = position
				} else if pair[1] == -1 {
					pair[1] = position
				} else {
					return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring facet spans three stations", decaderr.ErrUnsupported)
				}
			}
			if pair[1] < 0 {
				return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring facet has no paired stations", decaderr.ErrUnsupported)
			}
			gap := (pair[1] - pair[0] + len(rec.Outlines[0])) % len(rec.Outlines[0])
			if gap != 1 && gap != len(rec.Outlines[0])-1 {
				return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring facet skips a station", decaderr.ErrUnsupported)
			}
			if pair[0] > pair[1] {
				pair[0], pair[1] = pair[1], pair[0]
			}
			if dropPairs[pair] {
				dropCounts[pair]++
				continue
			}
		}
		mesh.Add(tri, group)
	}
	for pair := range dropPairs {
		if dropCounts[pair] != 2 {
			return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet ring cut loses a facet pair", decaderr.ErrUnsupported)
		}
	}
	retainedWalls := len(mesh.Tris)
	var replacement [2]map[int][]int
	var skip [2]map[int]bool
	for capIndex := range replacement {
		replacement[capIndex] = make(map[int][]int)
		skip[capIndex] = make(map[int]bool)
	}
	ringReplacement := make(map[int]int)
	arcBound := 0.0
	for _, contact := range contacts {
		if err := ctx.Err(); err != nil {
			return nil, outlines, 0, err
		}
		part := rec.Parts[contact.part]
		station := contact.station
		if station != 0 && station != part.RootStart {
			return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet lost a root station", decaderr.ErrUnsupported)
		}
		path, bound, err := filletBand(mesh, part, contact, radius)
		if err != nil {
			return nil, outlines, 0, err
		}
		arcBound = max(arcBound, bound)
		group := len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref,
			Role: fmt.Sprintf("fillet(%d,%d)", contact.part, station)}}})
		for capIndex := range 2 {
			old := part.Vertex[capIndex][station]
			last := path[capIndex][len(path[capIndex])-1]
			cut := contact.RingCut
			lastConsumed := rec.Outlines[capIndex][cut.Positions[len(cut.Positions)-1]]
			ringReplacement[lastConsumed] = last
			for _, position := range cut.Positions[1:] {
				skip[capIndex][rec.Outlines[capIndex][position]] = true
			}
			walk := path[capIndex]
			if station == 0 {
				walk = append([]int(nil), walk...)
				for i, j := 0, len(walk)-1; i < j; i, j = i+1, j-1 {
					walk[i], walk[j] = walk[j], walk[i]
				}
			}
			replacement[capIndex][old] = walk
		}
		for i := range len(path[0]) - 1 {
			b0, b1 := path[0][i], path[0][i+1]
			t0, t1 := path[1][i], path[1][i+1]
			mesh.Add([3]int{b0, b1, t1}, group)
			mesh.Add([3]int{b0, t1, t0}, group)
		}
	}
	// Only the original circular-wall facets use a root point as their end.
	// Rebind those endpoints to the circle foot; the connector wall still
	// ends at the line foot stored under the original vertex index.
	for i := range retainedWalls {
		if mesh.Src[i] != rec.Groups[0] {
			continue
		}
		for j, index := range mesh.Tris[i] {
			if replacement, ok := ringReplacement[index]; ok {
				mesh.Tris[i][j] = replacement
			}
		}
	}
	for capIndex, original := range rec.Outlines {
		for _, index := range original {
			if skip[capIndex][index] {
				continue
			}
			if expanded, ok := replacement[capIndex][index]; ok {
				outlines[capIndex] = append(outlines[capIndex], expanded...)
			} else {
				outlines[capIndex] = append(outlines[capIndex], index)
			}
		}
		points := make([]triangulation.Point2, len(outlines[capIndex]))
		loop := make([]int, len(points))
		for i, index := range outlines[capIndex] {
			v := mesh.Verts[index]
			points[i], loop[i] = triangulation.Point2{U: v.X, V: v.Y}, i
		}
		tris, err := triangulation.Triangulate(ctx, points, [][]int{loop})
		if err != nil {
			return nil, outlines, 0, triangulation.WrapLoftError(err)
		}
		for _, tri := range tris {
			mapped := [3]int{outlines[capIndex][tri[0]], outlines[capIndex][tri[1]],
				outlines[capIndex][tri[2]]}
			if capIndex == 0 {
				mapped[1], mapped[2] = mapped[2], mapped[1]
			}
			mesh.Add(mapped, rec.Groups[capIndex+1])
		}
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return nil, outlines, 0, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), mesh.Verts,
		mesh.Tris, loftmesh.MaxLoftAuditTriangles, loftcylinder.AuditMaxScans); err != nil {
		return nil, outlines, 0, err
	}
	for capIndex, outline := range outlines {
		z := rec.Source.Blank.Z0
		if capIndex == 1 {
			z = rec.Source.Blank.Z1
		}
		if !loftcylinder.AuditCap(ctx, mesh.Verts, mesh.Tris, mesh.Src, outline, rec.Groups[capIndex+1], z,
			capIndex == 0) ||
			!RootChordsInsideCap(ctx, mesh, outline, rec.Parts, capIndex) {
			return nil, outlines, 0, fmt.Errorf("%w: joined-prism fillet cap has no exact tiling or source-root proof",
				decaderr.ErrUnsupported)
		}
	}
	return mesh, outlines, arcBound, nil
}

func filletBand(mesh *facetedrecord.JoinedMesh, part facetedrecord.PrismPatternPart,
	contact FilletContact, radius float64) ([2][]int, float64, error) {
	var path [2][]int
	p, q, center := contact.line, contact.circle, contact.center
	start := math.Atan2(p.Y-center.Y, p.X-center.X)
	end := math.Atan2(q.Y-center.Y, q.X-center.X)
	sweep := math.Remainder(end-start, 2*math.Pi)
	if proofbound.IsNonFinite(sweep) || math.Abs(sweep) <= 0.01 || math.Abs(sweep) >= math.Pi ||
		contact.station == 0 && sweep <= 0 || contact.station == part.RootStart && sweep >= 0 {
		return path, 0, fmt.Errorf("%w: joined-prism fillet has no bounded corner sweep", decaderr.ErrUnsupported)
	}
	steps := int(math.Ceil(math.Abs(sweep) / 0.1))
	if steps < 1 || steps > 64 || contact.pointError >= radius/8 {
		return path, 0, fmt.Errorf("%w: joined-prism fillet exceeds its arc-sampling budget", decaderr.ErrUnsupported)
	}
	// Each endpoint radius vector differs from the true one by at most twice
	// the contact point error. Below r/4, its angle differs by at most 4e/r.
	angleError := proofbound.ProductUpper(4, proofbound.DivUpper(contact.pointError, radius))
	startIV := proofbound.Atan2Interval(
		new(big.Rat).Sub(proofarith.FloatRat(p.Y), proofarith.FloatRat(center.Y)),
		new(big.Rat).Sub(proofarith.FloatRat(p.X), proofarith.FloatRat(center.X)), false)
	endIV := proofbound.Atan2Interval(
		new(big.Rat).Sub(proofarith.FloatRat(q.Y), proofarith.FloatRat(center.Y)),
		new(big.Rat).Sub(proofarith.FloatRat(q.X), proofarith.FloatRat(center.X)), false)
	if end-start > math.Pi {
		endIV = proofbound.IntervalSub(endIV, proofbound.TwoPiInterval())
	} else if end-start < -math.Pi {
		endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
	}
	startIV = proofbound.IntervalWiden(startIV, proofarith.FloatRat(angleError))
	endIV = proofbound.IntervalWiden(endIV, proofarith.FloatRat(angleError))
	trueSweep := proofbound.IntervalSub(endIV, startIV)
	if sweep > 0 && (trueSweep.Lo.Sign() <= 0 ||
		trueSweep.Hi.Cmp(proofbound.PiLower) >= 0) ||
		sweep < 0 && (trueSweep.Hi.Sign() >= 0 ||
			new(big.Rat).Neg(trueSweep.Lo).Cmp(proofbound.PiLower) >= 0) {
		return path, 0, fmt.Errorf("%w: joined-prism fillet arc has no unique short sweep", decaderr.ErrUnsupported)
	}
	maxRound, maxAngleError := 0.0, 0.0
	for capIndex := range 2 {
		old := part.Vertex[capIndex][contact.station]
		z := mesh.Verts[old].Z
		mesh.Verts[old] = r3.Vec{X: p.X, Y: p.Y, Z: z}
		path[capIndex] = append(path[capIndex], old)
		for step := 1; step < steps; step++ {
			angle := start + float64(step)*sweep/float64(steps)
			fraction := big.NewRat(int64(step), int64(steps))
			angleIV := proofbound.IntervalAdd(
				proofbound.IntervalScale(startIV,
					new(big.Rat).Sub(big.NewRat(1, 1), fraction)),
				proofbound.IntervalScale(endIV, fraction))
			maxAngleError = max(maxAngleError, proofbound.IntervalFloatError(angleIV, angle))
			sine, cosine, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
			if !ok {
				return path, 0, fmt.Errorf("%w: joined-prism fillet angle has no trigonometric bound", decaderr.ErrUnsupported)
			}
			x := center.X + radius*math.Cos(angle)
			y := center.Y + radius*math.Sin(angle)
			v := r3.Vec{X: x, Y: y, Z: z}
			if !proofbound.FiniteVec(v) {
				return path, 0, fmt.Errorf("%w: joined-prism fillet has a non-finite arc station", decaderr.ErrUnsupported)
			}
			xIV := proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(center.X)),
				proofbound.IntervalScale(cosine, proofarith.FloatRat(radius)))
			yIV := proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(center.Y)),
				proofbound.IntervalScale(sine, proofarith.FloatRat(radius)))
			maxRound = max(maxRound, proofbound.AbsSumUpper(
				proofbound.IntervalFloatError(xIV, x), proofbound.IntervalFloatError(yIV, y)))
			path[capIndex] = append(path[capIndex], len(mesh.Verts))
			mesh.Verts = append(mesh.Verts, v)
		}
		path[capIndex] = append(path[capIndex], len(mesh.Verts))
		mesh.Verts = append(mesh.Verts, r3.Vec{X: q.X, Y: q.Y, Z: z})
	}
	maxAngleError = max(maxAngleError, proofbound.IntervalFloatError(startIV, start),
		proofbound.IntervalFloatError(endIV, start+sweep))
	stepAngle := proofbound.AbsSumUpper(proofbound.DivUpper(math.Abs(sweep), float64(steps)),
		proofbound.ProductUpper(2, maxAngleError))
	sag := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(stepAngle, stepAngle)/8)
	bound := proofbound.AbsSumUpper(contact.pointError,
		proofbound.ProductUpper(radius, maxAngleError), maxRound, sag)
	return path, bound, nil
}
