package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// These are source joints for one authenticated nonvertical Revolve side.
// They describe all fitted interior knots and fitted/radial-arc adjacencies
// whose exact side polynomial has a time root. Source cell classification,
// tool transitions and held-switch coverage remain separate gates.
type revolveLoftFacesNonverticalJoints struct {
	fitKnots []revolveLoftFacesFitKnotJoint
	radial   []revolveLoftFacesRadialJoint
	fitLines []revolveLoftFacesFitLineJoint
}

func revolveLoftFacesEnumerateNonverticalJoints(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan) (
	revolveLoftFacesNonverticalJoints, bool, error,
) {
	var out revolveLoftFacesNonverticalJoints
	if ctx == nil || trim == nil || trim.Source == nil || trim.Source.source == nil ||
		section.start.U == section.end.U || section.start.V <= 0 || section.end.V <= 0 {
		return out, false, nil
	}
	segments := trim.Source.profile.Outer.Segments
	entities := trim.Source.source.entities
	if len(segments) == 0 || len(segments) != len(entities) {
		return out, false, nil
	}
	for segmentIndex, source := range segments {
		if err := ctx.Err(); err != nil {
			return out, false, err
		}
		if _, ok := source.(fitSplineSeg); !ok {
			continue
		}
		spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segmentIndex, source)
		if !ok {
			return out, false, nil
		}
		for left := 0; left+1 < len(spans); left++ {
			vertex, ok := revolveLoftFacesFitKnotVertex(trim, segmentIndex, left)
			if !ok {
				return out, false, nil
			}
			for band := 0; band+1 < len(trim.Maps); band++ {
				roots, ok := revolveLoftFacesEndpointFiniteRootsWith(revolveLoftFacesMeridianMemoOf(ctx), trim, section,
					vertex.left, band, true)
				if !ok {
					return out, false, fmt.Errorf("%w: fitted source knot %d/%d band %d has undecidable time roots",
						ErrUnsupported, segmentIndex, left, band)
				}
				if len(roots) == 0 {
					continue
				}
				joint, ok := certifyRevolveLoftFacesFitKnotJoint(ctx, trim,
					section, segmentIndex, left, band)
				if !ok {
					return out, false, fmt.Errorf("%w: fitted source knot %d/%d band %d lacks a joint cell proof",
						ErrUnsupported, segmentIndex, left, band)
				}
				out.fitKnots = append(out.fitKnots, joint)
			}
		}
		for _, fitAtEnd := range [...]bool{false, true} {
			arcIndex := (segmentIndex + 1) % len(segments)
			if !fitAtEnd {
				arcIndex = (segmentIndex - 1 + len(segments)) % len(segments)
			}
			if _, isLine := entities[arcIndex].(*sketch.Line); isLine {
				vertex, valid := revolveLoftFacesFitLineVertexAt(trim,
					segmentIndex, arcIndex, fitAtEnd)
				if !valid {
					return out, false, nil
				}
				for band := 0; band+1 < len(trim.Maps); band++ {
					roots, valid := revolveLoftFacesEndpointFiniteRootsWith(revolveLoftFacesMeridianMemoOf(ctx), trim, section,
						vertex.fitSpan, band, vertex.fitAtEnd)
					if !valid {
						return out, false, fmt.Errorf("%w: fit/line source joint %d/%d band %d has undecidable time roots",
							ErrUnsupported, segmentIndex, arcIndex, band)
					}
					if len(roots) == 0 {
						continue
					}
					joint, valid := certifyRevolveLoftFacesFitLineJoint(ctx, trim,
						section, segmentIndex, arcIndex, band, fitAtEnd)
					if !valid || joint.vertex.point != vertex.point {
						return out, false, fmt.Errorf("%w: fit/line source joint %d/%d band %d lacks a joint cell proof",
							ErrUnsupported, segmentIndex, arcIndex, band)
					}
					out.fitLines = append(out.fitLines, joint)
				}
				continue
			}
			if _, isArc := entities[arcIndex].(*sketch.Arc); !isArc {
				continue
			}
			vertex, ok := revolveLoftFacesFitArcVertex(trim, segmentIndex, arcIndex, fitAtEnd)
			if !ok {
				return out, false, nil
			}
			for band := 0; band+1 < len(trim.Maps); band++ {
				radial, ok := mappedArcRadialTimeBand(ctx, trim, arcIndex, band, section)
				if !ok {
					continue // a nonradial arc is classified by its own source cells
				}
				q := [3][]*big.Rat{{radial.q[0]}, {radial.q[1]}, {radial.q[2]}}
				roots, ok := loftface.EndpointTimeRoots(q, false)
				if !ok {
					return out, false, fmt.Errorf("%w: radial source arc %d band %d has undecidable time roots",
						ErrUnsupported, arcIndex, band)
				}
				if len(roots) == 0 {
					continue
				}
				joint, ok := certifyRevolveLoftFacesRadialJoint(ctx, trim,
					section, segmentIndex, arcIndex, band, fitAtEnd)
				if !ok || joint.vertex.point != vertex.point {
					return out, false, fmt.Errorf("%w: radial source arc %d band %d lacks a fitted joint proof",
						ErrUnsupported, arcIndex, band)
				}
				out.radial = append(out.radial, joint)
			}
		}
	}
	return out, true, nil
}

func revolveLoftFacesRadialPartitionCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	joint revolveLoftFacesRadialJoint) ([]revolveLoftFacesMappedCell, bool, error) {
	if joint.vertex.fitSegment != segment || joint.radial.band != band ||
		len(joint.fitCells) != len(joint.roots) {
		return nil, false, nil
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment,
		trim.Source.profile.Outer.Segments[segment])
	if !ok || spanIndex < 0 || spanIndex >= len(spans) ||
		joint.vertex.fitAtEnd && spanIndex != len(spans)-1 ||
		!joint.vertex.fitAtEnd && spanIndex != 0 ||
		len(span) != len(joint.vertex.fitSpan) {
		return nil, false, nil
	}
	for i := range span {
		if span[i].U.Cmp(joint.vertex.fitSpan[i].U) != 0 ||
			span[i].V.Cmp(joint.vertex.fitSpan[i].V) != 0 {
			return nil, false, nil
		}
	}
	cells, certified, err := revolveLoftFacesEndpointPartitionCells(ctx, trim, section, segment,
		spanIndex, band, span, joint.roots, joint.pieces,
		joint.fitCells, joint.vertex.fitAtEnd)
	for i := range cells {
		if cells[i].side == revolveLoftFacesJointContact {
			cells[i].jointKind = revolveLoftFacesRadialArcContact
			cells[i].jointPoint = joint.vertex.point
		}
	}
	return cells, certified, err
}
