package prismboolean

import (
	"context"
	"strings"

	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayCapBudget carries the shared proof and allocation meter.
type ReplayCapBudget interface {
	proof.ReplayKeyBudget
	ReserveArray(context.Context, uint64, uint64) error
}

type ReplayGroupCapFaces struct{ Wall, Start, End *topo.Face }

// ReplayGroupCapStructure checks role, body, loop incidence and cap-record
// fields. Plane identity and the section's geometric clearance remain separate.
func ReplayGroupCapStructure(ctx context.Context, body *topo.Body, budget ReplayCapBudget) ([]ReplayGroupCapFaces, bool, error) {
	if body == nil || budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	sp, ok := body.Payload.(stackedrecord.Payload)
	if !ok || !sp.IsGroup() || sp.ClosedShell {
		return nil, false, nil
	}
	slab := sp.Slabs[0]
	sealed, err := body.ReplayPrismBuildPayload(ctx, sp.Frame, sp.Xform, slab.Z0, slab.Z1,
		slab.Z0Delta, slab.Z1Delta, sp.SectionDelta, slab.Z0Denoted, slab.Z1Denoted, slab.Regions, budget)
	if err != nil || !sealed {
		return nil, false, err
	}
	return replayCapStructure(ctx, body, sp, true, budget)
}

// ReplayPrismCapStructure binds a circle prism's wall and cap faces.
// Plane identity and section clearance remain separate gates.
func ReplayPrismCapStructure(ctx context.Context, body *topo.Body, budget ReplayCapBudget) ([]ReplayGroupCapFaces, bool, error) {
	if budget == nil || body == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveBytes(ctx, 512); err != nil {
		return nil, false, err
	}
	sealed, err := body.ReplayPrismBuildPayload(ctx, pp.Frame, pp.Xform, pp.Z0, pp.Z1,
		pp.Z0Delta, pp.Z1Delta, pp.SectionDelta, pp.Z0Denoted, pp.Z1Denoted,
		[]momentinput.Profile{pp.Profile}, budget)
	if err != nil || !sealed {
		return nil, false, err
	}
	if err := budget.ReserveRecords(ctx, 2); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveBytes(ctx, 2048); err != nil {
		return nil, false, err
	}
	slab := stackedrecord.Slab{Regions: []momentinput.Profile{pp.Profile}, Z0: pp.Z0, Z1: pp.Z1,
		Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta, Z0Denoted: pp.Z0Denoted, Z1Denoted: pp.Z1Denoted}
	return replayCapStructure(ctx, body, stackedrecord.Payload{Frame: pp.Frame, Xform: pp.Xform, Slabs: []stackedrecord.Slab{slab}}, false, budget)
}

func replayCapStructure(ctx context.Context, body *topo.Body, sp stackedrecord.Payload, group bool, budget ReplayCapBudget) ([]ReplayGroupCapFaces, bool, error) {
	if budget == nil || body == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if len(sp.Slabs) != 1 || len(sp.Interfaces) != 0 || (group && !sp.IsGroup()) || (!group && len(sp.Slabs[0].Regions) != 1) {
		return nil, false, nil
	}
	n := len(sp.Slabs[0].Regions)
	if n > 4096 {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), 24); err != nil {
		return nil, false, err
	}
	result := make([]ReplayGroupCapFaces, n)
	// Capture side faces first. Every admitted region is one whole circle.
	faceCount := 0
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if lump == nil {
			return nil, false, nil
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			if shell == nil {
				return nil, false, nil
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, false, err
				}
				faceCount++
				if face == nil || face.Body != body || len(face.Origins) != 1 || face.Origins[0].Producer != body.Origin.Producer {
					return nil, false, nil
				}
				built, err := replayBuiltFace(ctx, body, face, budget)
				if err != nil || !built {
					return nil, false, err
				}

				role := face.Origins[0].Role
				index := 0
				if !group {
					if role != "side(0,0)" {
						if role != provenance.RoleCapStart && role != provenance.RoleCapEnd {
							return nil, false, nil
						}
						continue
					}
				} else {
					suffix, yes := strings.CutPrefix(role, "slab(0).region(")
					if !yes {
						if role != provenance.RoleCapStart && role != provenance.RoleCapEnd {
							return nil, false, nil
						}
						continue
					}
					digits, yes := strings.CutSuffix(suffix, ").side(0,0)")
					if !yes || len(digits) == 0 || (len(digits) > 1 && digits[0] == '0') {
						return nil, false, nil
					}
					for _, d := range digits {
						if err := budget.SpendWork(ctx, 1); err != nil {
							return nil, false, err
						}
						if d < '0' || d > '9' || index > (n-int(d-'0'))/10 {
							return nil, false, nil
						}
						index = index*10 + int(d-'0')
					}
				}
				if index >= n || result[index].Wall != nil {
					return nil, false, nil
				}
				cylinder, yes := face.Surface.(surfacegeom.Cylinder)
				region := sp.Slabs[0].Regions[index]
				if !yes || face.CapRecord != nil || len(face.Loops) != 2 || len(region.Holes) != 0 || len(region.Outer.Segments) != 1 {
					return nil, false, nil
				}
				circle, yes := region.Outer.Segments[0].(sectionrecord.CircleSeg)
				if !yes || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 || face.Reversed {
					return nil, false, nil
				}
				radius := circle.Radius.Base()
				if math.IsNaN(radius) || math.IsInf(radius, 0) || radius <= 0 {
					return nil, false, nil
				}
				if err := budget.SpendWork(ctx, 512); err != nil {
					return nil, false, err
				}
				proxy := prismrecord.Payload{Frame: sp.Frame, Xform: sp.Xform}
				if cylinder.Radius.Base() != radius || cylinder.Axis != proxy.Dir(0, 0, 1) || cylinder.Origin != proxy.Point(circle.Center.U, circle.Center.V, sp.Slabs[0].Z0) {
					return nil, false, nil
				}

				if face.Loops[0] == nil || face.Loops[1] == nil || len(face.Loops[0].CoEdges) != 1 || len(face.Loops[1].CoEdges) != 1 || face.Loops[0].CoEdges[0].Edge == nil || face.Loops[1].CoEdges[0].Edge == nil || face.Loops[0].CoEdges[0].Edge == face.Loops[1].CoEdges[0].Edge {
					return nil, false, nil
				}
				result[index].Wall = face
			}
		}
	}
	if faceCount != 3*n {
		return nil, false, nil
	}
	slab := sp.Slabs[0]
	if slab.Z0Delta != 0 || slab.Z1Delta != 0 {
		return nil, false, nil
	}
	proxy := prismrecord.Payload{Frame: sp.Frame, Xform: sp.Xform}
	for i := range result {
		wall := result[i].Wall
		if wall == nil {
			return nil, false, nil
		}
		for _, lump := range body.Lumps {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			for _, shell := range lump.Shells {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, false, err
				}
				for _, face := range shell.Faces {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return nil, false, err
					}
					role := face.Origins[0].Role
					if role != provenance.RoleCapStart && role != provenance.RoleCapEnd {
						continue
					}
					if len(face.Loops) != 1 || face.Loops[0] == nil || !face.Loops[0].Outer || len(face.Loops[0].CoEdges) != 1 || face.Loops[0].CoEdges[0].Edge == nil {
						return nil, false, nil
					}
					edge := face.Loops[0].CoEdges[0].Edge
					if edge != wall.Loops[0].CoEdges[0].Edge && edge != wall.Loops[1].CoEdges[0].Edge {
						continue
					}
					if len(edge.Faces) != 2 || (edge.Faces[0] != face || edge.Faces[1] != wall) && (edge.Faces[1] != face || edge.Faces[0] != wall) {
						return nil, false, nil
					}
					for previous := range i {
						if err := budget.SpendWork(ctx, 1); err != nil {
							return nil, false, err
						}
						if result[previous].Start == face || result[previous].End == face {
							return nil, false, nil
						}
					}

					top := role == provenance.RoleCapEnd
					z := slab.Z0
					if top {
						z = slab.Z1
					}
					capRecord := face.CapRecord
					if capRecord == nil || capRecord.Frame != sp.Frame || capRecord.Xform != sp.Xform || capRecord.Z != z || capRecord.ZDenoted == nil || !face.HasAxialDelta || face.AxialDelta != 0 || face.Reversed {
						return nil, false, nil
					}
					q, err := proof.ReplayFloatRat(ctx, z, budget)
					if err != nil {
						return nil, false, err
					}
					cmp, err := proof.ReplayRatCompare(ctx, capRecord.ZDenoted, q, budget)
					if err != nil {
						return nil, false, err
					}
					if cmp != 0 {
						return nil, false, nil
					}
					plane, yes := face.Surface.(surfacegeom.Plane)
					if !yes {
						return nil, false, nil
					}
					// The fixed-size float frame routine is precharged before it runs.
					if err := budget.SpendWork(ctx, 1024); err != nil {
						return nil, false, err
					}
					if err := budget.ReserveRecords(ctx, 8); err != nil {
						return nil, false, err
					}
					if err := budget.ReserveBytes(ctx, 4096); err != nil {
						return nil, false, err
					}
					expected, err := proxy.CapFrame(z, !top)
					if err != nil {
						return nil, false, err
					}
					if plane.Frame != expected {
						return nil, false, nil
					}
					if top {
						if result[i].End != nil {
							return nil, false, nil
						}
						result[i].End = face
					} else {
						if result[i].Start != nil {
							return nil, false, nil
						}
						result[i].Start = face
					}
				}
			}
		}
		if result[i].Start == nil || result[i].End == nil || result[i].Start == result[i].End {
			return nil, false, nil
		}
	}
	return result, true, nil
}
