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
	"github.com/lestrrat-3d/decad/internal/topo"
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
		blankGroup[group] = len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetGroup{Origins: face.originRefs(), Planar: group != 0})
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
		part.Vertex = make([][]int, 2)
		for capIndex := range 2 {
			part.Vertex[capIndex] = make([]int, part.RootStart+1)
			for j := range part.Vertex[capIndex] {
				part.Vertex[capIndex][j] = len(mesh.Verts)
				mesh.Verts = append(mesh.Verts, part.Mesh.Vertices[2*j+capIndex])
			}
		}
		groups := make(map[*topo.Face]int)
		for j := range part.RootStart {
			for side := range 2 {
				k := 2*j + side
				face := part.Mesh.Source[k]
				group, ok := groups[face]
				if !ok {
					group = len(mesh.Groups)
					groups[face] = group
					mesh.Groups = append(mesh.Groups, facetGroup{Origins: append([]provenance.Ref(nil), face.Origins...)})
				}
				old := part.Mesh.Triangles[k]
				var mapped [3]int
				for corner, index := range old {
					station, capIndex := index/2, index%2
					if station > part.RootStart {
						return nil, [2][]int{}, blankGroup, 0, 0,
							fmt.Errorf("%w: retained prism wall enters its root arc", ErrUnsupported)
					}
					mapped[corner] = part.Vertex[capIndex][station]
				}
				mesh.Add(mapped, group)
			}
		}
	}
	radius := source.BlankRadius
	maxStep, ringRound := 0.0, 0.0
	for i := range parts {
		part, next := &parts[i], &parts[(i+1)%len(parts)]
		for cap := range 2 {
			outlines[cap] = append(outlines[cap], part.Vertex[cap]...)
		}
		start, end := part.End, next.Start
		startIV, endIV := part.EndIV, next.StartIV
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
			rings[cap][0] = part.Vertex[cap][part.RootStart]
			rings[cap][steps] = next.Vertex[cap][0]
			for step := 1; step < steps; step++ {
				fraction := big.NewRat(int64(step), int64(steps))
				angleIV := proofbound.IntervalAdd(
					proofbound.IntervalScale(startIV,
						new(big.Rat).Sub(big.NewRat(1, 1), fraction)),
					proofbound.IntervalScale(endIV, fraction))
				angle := start + float64(step)*width/float64(steps)
				z := source.Blank.Z0
				if cap == 1 {
					z = source.Blank.Z1
				}
				point, round, err := prismpattern.RingPoint(radius, z, angle, angleIV)
				if err != nil {
					return nil, [2][]int{}, blankGroup, 0, 0, err
				}
				ringRound = max(ringRound, round)
				rings[cap][step] = len(mesh.Verts)
				mesh.Verts = append(mesh.Verts, point)
				outlines[cap] = append(outlines[cap], rings[cap][step])
			}
		}
		for step := range steps {
			b0, b1 := rings[0][step], rings[0][step+1]
			t0, t1 := rings[1][step], rings[1][step+1]
			if !loftcylinder.RingOrder(mesh.Verts[b0], mesh.Verts[b1],
				mesh.Verts[t0], mesh.Verts[t1]) {
				return nil, [2][]int{}, blankGroup, 0, 0,
					fmt.Errorf("%w: circular prism ring loses angular order", ErrUnsupported)
			}
			mesh.Add([3]int{b0, b1, t1}, blankGroup[0])
			mesh.Add([3]int{b0, t1, t0}, blankGroup[0])
		}
	}
	for cap, outline := range outlines {
		points := make([]triangulation.Point2, len(outline))
		loop := make([]int, len(outline))
		for i, index := range outline {
			point := mesh.Verts[index]
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
			mesh.Add(mapped, blankGroup[cap+1])
		}
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, [2][]int{}, blankGroup, 0, 0, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, [2][]int{}, blankGroup, 0, 0, err
	}
	for cap, outline := range outlines {
		z := source.Blank.Z0
		if cap == 1 {
			z = source.Blank.Z1
		}
		if !loftCylinderAuditCap(ctx, mesh, outline, blankGroup[cap+1], z, cap == 0) ||
			!prismpattern.RootChordsInsideCap(ctx, mesh, outline, parts, cap) {
			if err := ctx.Err(); err != nil {
				return nil, [2][]int{}, blankGroup, 0, 0, err
			}
			return nil, [2][]int{}, blankGroup, 0, 0,
				fmt.Errorf("%w: circular prism cap does not enclose its source roots", ErrUnsupported)
		}
	}
	return mesh, outlines, blankGroup, maxStep, ringRound, nil
}
