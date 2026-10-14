package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// prismPatternShell replaces only the inward root walls with exposed pieces
// of the source blank's circular wall. Every retained side comes from a
// verified source mesh; both caps share exactly the side boundary vertices.
func prismPatternShell(ctx context.Context, blank *Body, source *prismPatternSource,
	parts []prismPatternPart) (*joinedMesh, [2][]int, [3]int, float64, float64, error) {
	mesh := &joinedMesh{}
	blankGroup := [3]int{-1, -1, -1}
	for _, face := range blank.Faces() {
		if len(face.Origins()) == 0 {
			return nil, [2][]int{}, blankGroup, 0, 0,
				fmt.Errorf("%w: circular blank lost a source role", ErrUnsupported)
		}
		group := -1
		switch face.Origins()[0].Role {
		case prismrecord.OuterFirstSideRole:
			group = 0
		case provenance.RoleCapStart:
			group = 1
		case provenance.RoleCapEnd:
			group = 2
		}
		if group < 0 || blankGroup[group] >= 0 {
			return nil, [2][]int{}, blankGroup, 0, 0,
				fmt.Errorf("%w: circular blank changed its three source faces", ErrUnsupported)
		}
		blankGroup[group] = len(mesh.groups)
		mesh.groups = append(mesh.groups, facetGroup{origins: face.originRefs(), planar: group != 0})
	}
	for _, group := range blankGroup {
		if group < 0 {
			return nil, [2][]int{}, blankGroup, 0, 0,
				fmt.Errorf("%w: circular blank lost a source face", ErrUnsupported)
		}
	}
	var outlines [2][]int
	for i := range parts {
		part := &parts[i]
		part.vertex = make([][]int, 2)
		for capIndex := range 2 {
			part.vertex[capIndex] = make([]int, part.rootStart+1)
			for j := range part.vertex[capIndex] {
				part.vertex[capIndex][j] = len(mesh.verts)
				mesh.verts = append(mesh.verts, part.mesh.n.Vertices[2*j+capIndex])
			}
		}
		groups := make(map[*Face]int)
		for j := range part.rootStart {
			for side := range 2 {
				k := 2*j + side
				face := part.mesh.source[k]
				group, ok := groups[face]
				if !ok {
					group = len(mesh.groups)
					groups[face] = group
					mesh.groups = append(mesh.groups, facetGroup{origins: face.originRefs()})
				}
				old := part.mesh.n.Triangles[k]
				var mapped [3]int
				for corner, index := range old {
					station, capIndex := index/2, index%2
					if station > part.rootStart {
						return nil, [2][]int{}, blankGroup, 0, 0,
							fmt.Errorf("%w: retained prism wall enters its root arc", ErrUnsupported)
					}
					mapped[corner] = part.vertex[capIndex][station]
				}
				mesh.add(mapped, group)
			}
		}
	}
	radius := source.blankRadius
	maxStep, ringRound := 0.0, 0.0
	for i := range parts {
		part, next := &parts[i], &parts[(i+1)%len(parts)]
		for cap := range 2 {
			outlines[cap] = append(outlines[cap], part.vertex[cap]...)
		}
		start, end := part.end, next.start
		startIV, endIV := part.endIV, next.startIV
		for end <= start {
			end += 2 * math.Pi
			endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
		}
		width := end - start
		steps := int(math.Ceil(width / 0.02))
		if steps < 1 || steps > 512 {
			return nil, [2][]int{}, blankGroup, 0, 0,
				fmt.Errorf("%w: circular blank gap exceeds its chord budget", ErrUnsupported)
		}
		maxStep = max(maxStep, width/float64(steps))
		var rings [2][]int
		for cap := range 2 {
			rings[cap] = make([]int, steps+1)
			rings[cap][0] = part.vertex[cap][part.rootStart]
			rings[cap][steps] = next.vertex[cap][0]
			for step := 1; step < steps; step++ {
				fraction := big.NewRat(int64(step), int64(steps))
				angleIV := proofbound.IntervalAdd(
					proofbound.IntervalScale(startIV,
						new(big.Rat).Sub(big.NewRat(1, 1), fraction)),
					proofbound.IntervalScale(endIV, fraction))
				angle := start + float64(step)*width/float64(steps)
				z := source.blank.Z0
				if cap == 1 {
					z = source.blank.Z1
				}
				point, round, err := prismpattern.RingPoint(radius, z, angle, angleIV)
				if err != nil {
					return nil, [2][]int{}, blankGroup, 0, 0, err
				}
				ringRound = max(ringRound, round)
				rings[cap][step] = len(mesh.verts)
				mesh.verts = append(mesh.verts, point)
				outlines[cap] = append(outlines[cap], rings[cap][step])
			}
		}
		for step := range steps {
			b0, b1 := rings[0][step], rings[0][step+1]
			t0, t1 := rings[1][step], rings[1][step+1]
			if !loftcylinder.RingOrder(mesh.verts[b0], mesh.verts[b1],
				mesh.verts[t0], mesh.verts[t1]) {
				return nil, [2][]int{}, blankGroup, 0, 0,
					fmt.Errorf("%w: circular prism ring loses angular order", ErrUnsupported)
			}
			mesh.add([3]int{b0, b1, t1}, blankGroup[0])
			mesh.add([3]int{b0, t1, t0}, blankGroup[0])
		}
	}
	for cap, outline := range outlines {
		points := make([]triangulation.Point2, len(outline))
		loop := make([]int, len(outline))
		for i, index := range outline {
			point := mesh.verts[index]
			points[i] = triangulation.Point2{U: point.X, V: point.Y}
			loop[i] = i
		}
		tris, err := triangulation.Triangulate(ctx, points, [][]int{loop})
		if err != nil {
			return nil, [2][]int{}, blankGroup, 0, 0, triangulation.WrapLoftError(err)
		}
		for _, tri := range tris {
			mapped := [3]int{outline[tri[0]], outline[tri[1]], outline[tri[2]]}
			if cap == 0 {
				mapped[1], mapped[2] = mapped[2], mapped[1]
			}
			mesh.add(mapped, blankGroup[cap+1])
		}
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, [2][]int{}, blankGroup, 0, 0, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, [2][]int{}, blankGroup, 0, 0, err
	}
	for cap, outline := range outlines {
		z := source.blank.Z0
		if cap == 1 {
			z = source.blank.Z1
		}
		if !loftCylinderAuditCap(ctx, mesh, outline, blankGroup[cap+1], z, cap == 0) ||
			!prismPatternRootChordsInsideCap(ctx, mesh, outline, parts, cap) {
			if err := ctx.Err(); err != nil {
				return nil, [2][]int{}, blankGroup, 0, 0, err
			}
			return nil, [2][]int{}, blankGroup, 0, 0,
				fmt.Errorf("%w: circular prism cap does not enclose its source roots", ErrUnsupported)
		}
	}
	return mesh, outlines, blankGroup, maxStep, ringRound, nil
}
