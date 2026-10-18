package decad

import (
	"context"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryPrismPatternUnionMany admits only an authenticated circular prism and
// rigid copies of one extruded section. Each later proof stage may refuse;
// the public Union commits only a fully audited result.
func tryPrismPatternUnionMany(ctx context.Context, d *Document, ref producerID,
	operands []*Body) (*Body, bool, error) {
	source, matched, err := prismpattern.SourceOf(ctx, bodyNodes(operands))
	if !matched || err != nil {
		return nil, matched, err
	}
	blank := source.Blank
	if blank.SurfaceResult || blank.SectionDelta != 0 || blank.Z0Delta != 0 || blank.Z1Delta != 0 ||
		blank.Frame.Origin() != (r3.Vec{}) || blank.Frame.U() != (r3.Vec{X: 1}) ||
		blank.Frame.V() != (r3.Vec{Y: 1}) || blank.Z0 != 0 || blank.Z1 <= 0 ||
		source.RootIndex != len(source.Section.Profile.Outer.Segments)-1 ||
		source.RootArc.TStart != 1 || source.RootArc.TEnd != 0 {
		return nil, true, fmt.Errorf("%w: copied-prism circular join needs a positive axial blank and trailing inward root arc",
			ErrUnsupported)
	}
	parts := make([]prismPatternPart, len(source.Sections))
	for i, body := range source.Sections {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		p, ok := body.Payload.(prismPayload)
		if !ok {
			return nil, true, fmt.Errorf("%w: copied-prism section %d lost its prism payload", ErrUnsupported, i+1)
		}
		if p.SurfaceResult || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 {
			return nil, true, fmt.Errorf("%w: copied-prism section %d has a displaced or open source", ErrUnsupported, i+1)
		}
		m, err := bodyHandle(body).Tessellate(ctx, units.Millimeters(0.02), WithVerification(VerifyAll))
		if err != nil {
			return nil, true, err
		}
		if !m.BoundaryVerified() || !m.VolumeVerified() {
			return nil, true, fmt.Errorf("%w: copied-prism section %d lacks a complete mesh proof", ErrUnsupported, i+1)
		}
		rootStart, err := prismPatternMeshRoot(m, bodyHandle(body), source.RootIndex, blank.Z0, blank.Z1)
		if err != nil {
			return nil, true, err
		}
		first := m.n.Vertices[0]
		last := m.n.Vertices[2*rootStart]
		start, end := math.Atan2(first.Y, first.X), math.Atan2(last.Y, last.X)
		startIV := proofbound.Atan2Interval(proofarith.FloatRat(first.Y), proofarith.FloatRat(first.X), false)
		endIV := proofbound.Atan2Interval(proofarith.FloatRat(last.Y), proofarith.FloatRat(last.X), false)
		for end <= start {
			end += 2 * math.Pi
			endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
		}
		if end-start <= 0 || end-start >= math.Pi/2 ||
			startIV.Hi.Cmp(endIV.Lo) >= 0 {
			return nil, true, fmt.Errorf("%w: copied-prism section %d has no ordered root sector", ErrUnsupported, i+1)
		}
		parts[i] = prismPatternPart{Body: body, Prism: p, Mesh: meshNode(m), RootStart: rootStart,
			Start: start, End: end, StartIV: startIV, EndIV: endIV}
	}
	sort.Slice(parts, func(i, j int) bool { return parts[i].Start < parts[j].Start })
	for i := range parts {
		for j := i + 1; j < len(parts); j++ {
			if !prismpattern.MeshSeparated(parts[i], parts[j]) {
				return nil, true, fmt.Errorf("%w: copied-prism sections %d and %d have no certified separating plane",
					ErrUnsupported, i+1, j+1)
			}
		}
	}
	for i := range parts {
		a, b := &parts[i], &parts[(i+1)%len(parts)]
		nextStart := b.Start
		nextIV := b.StartIV
		for nextStart <= a.End {
			nextStart += 2 * math.Pi
			nextIV = proofbound.IntervalAdd(nextIV, proofbound.TwoPiInterval())
		}
		if nextStart-a.End >= 2*math.Pi || a.EndIV.Hi.Cmp(nextIV.Lo) >= 0 {
			return nil, true, fmt.Errorf("%w: copied-prism root sectors have no ordered circular gap", ErrUnsupported)
		}
	}
	mesh, outlines, groups, step, round, err := prismPatternShell(ctx, operands[0], source, parts)
	if err != nil {
		return nil, true, err
	}
	body, err := prismPatternJoinedBody(ctx, d, ref, operands[0], source, parts,
		mesh, outlines, groups, step, round)
	return body, true, err
}

func prismPatternMeshRoot(m *Mesh, body *Body, rootIndex int, z0, z1 float64) (int, error) {
	role := fmt.Sprintf("side(0,%d)", rootIndex)
	var roots []*topo.Face
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			if origin.Role == role {
				roots = append(roots, faceNode(face))
			}
		}
	}
	return tessellation.PrismPatternMeshRoot(m.n.Vertices, len(m.n.Triangles), m.n.Source, roots, z0, z1)
}
