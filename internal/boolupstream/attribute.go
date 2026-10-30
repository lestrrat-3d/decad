// Package boolupstream attributes an analytic Boolean result's planar faces
// to the operand faces they lie inside, so the operands' origin roles survive
// the Boolean (agent-docs/prism-boolean-design.md §11). It only reads exact
// plane identity, never a residual: a face it cannot decide inherits nothing.
package boolupstream

import (
	"context"
	"fmt"
	"math/big"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capplane"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Attach records on result, which op built from operands a and b, the
// operand roles each of its planar faces inherits (Attribute), in the result
// payload's Upstream field and on the faces themselves. A result whose
// payload is not a prism, stacked or brep record carries no Upstream field
// and attaches nothing. It returns ctx.Err() when ctx is canceled.
func Attach(ctx context.Context, op meshbool.OperationKind, result, a, b *topo.Body) error {
	if !carries(result.Payload) {
		return nil
	}
	table, err := Attribute(ctx, op, result, a, b)
	if err != nil || len(table) == 0 {
		return err
	}
	switch p := result.Payload.(type) {
	case prismrecord.Payload:
		p.Upstream = table
		result.Payload = p
	case stackedrecord.Payload:
		p.Upstream = table
		result.Payload = p
	case breprecord.Payload:
		p.Upstream = table
		result.Payload = p
	}
	table.Apply(result)
	return nil
}

func carries(p topo.Payload) bool {
	switch p.(type) {
	case prismrecord.Payload, stackedrecord.Payload, breprecord.Payload:
		return true
	default:
		return false
	}
}

// Attribute returns the upstream roles of result's faces. A result face F
// whose CapRecord gives it an exact plane P (capplane.OfFace) and an outward
// side s inherits the origins, own and upstream, of operand face G when G is
// the ONLY operand face lying on P with the side F's points need: s for a
// Union or Intersect operand and for a Cut's target, -s for a Cut's tool. A
// result's boundary lies in its operands' boundaries, and a face on P can
// meet any face off P only in a set of zero area, so F then lies inside G.
//
// An operand face counts as off P only by an exact argument: a CapRecord face
// with a Plane tag whose record normal is not parallel to P's, or whose exact
// plane is another plane; or a swept wall of a prism, stacked or brep record
// whose sweep direction crosses P. Any operand face it cannot place that way —
// a displaced face with no denoted level, a Faceted or retained tag, a wall
// parallel to P, a band patch, a face of another payload class with no
// CapRecord — leaves F unattributed. Faces sharing one own-role list must all
// inherit the same roles, or none of them does, since the table is keyed by
// role.
func Attribute(ctx context.Context, op meshbool.OperationKind, result, a, b *topo.Body) (topo.UpstreamRoles, error) {
	operands := [2]operand{{body: a}, {body: b, flip: op == meshbool.OpCut}}
	for i := range operands {
		operands[i].faces = facesOf(operands[i].body)
		operands[i].axis, operands[i].axisOK = recordAxis(operands[i].body.Payload)
	}
	type row struct {
		roles   []string
		origins []provenance.Ref
		ok      bool
	}
	var rows []*row
	byKey := map[string]*row{}
	for _, f := range facesOf(result) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if len(f.Origins) == 0 {
			continue
		}
		roles := make([]string, len(f.Origins))
		for i, o := range f.Origins {
			roles[i] = o.Role
		}
		key := strings.Join(roles, "\x00")
		origins, ok := attribute(f, operands)
		r, seen := byKey[key]
		if !seen {
			r = &row{roles: roles, origins: origins, ok: ok}
			byKey[key] = r
			rows = append(rows, r)
			continue
		}
		if !ok || !r.ok || !slices.Equal(origins, r.origins) {
			r.ok = false
		}
	}
	var table topo.UpstreamRoles
	for _, r := range rows {
		if r.ok {
			table = append(table, topo.UpstreamEntry{Roles: r.roles, Origins: r.origins})
		}
	}
	return table, nil
}

// operand is one Boolean operand: its body, its faces, whether its faces
// bound the result from the other side (a Cut's tool), and the one sweep axis
// of a prism or stacked record (axisOK false for any other).
type operand struct {
	body   *topo.Body
	faces  []*topo.Face
	flip   bool
	axis   [3]*big.Rat
	axisOK bool
}

// attribute returns the roles F inherits, ok=false when it inherits none.
func attribute(f *topo.Face, operands [2]operand) ([]provenance.Ref, bool) {
	plane, ok := capplane.OfFace(f)
	if !ok {
		return nil, false
	}
	side, ok := outwardSense(plane, f)
	if !ok {
		return nil, false
	}
	var found *topo.Face
	for _, o := range operands {
		want := side
		if o.flip {
			want = -side
		}
		for _, g := range o.faces {
			on, ok := onPlane(plane, want, o, g)
			if !ok {
				return nil, false
			}
			if !on {
				continue
			}
			if found != nil {
				return nil, false
			}
			found = g
		}
	}
	if found == nil {
		return nil, false
	}
	var out []provenance.Ref
	for _, ref := range found.AllOrigins() {
		if !slices.Contains(out, ref) {
			out = append(out, ref)
		}
	}
	return out, len(out) > 0
}

// onPlane decides whether operand face g lies on plane with outward side
// want. ok=false means the question is undecided.
func onPlane(plane capplane.Plane, want int, o operand, g *topo.Face) (bool, bool) {
	if g.CapRecord != nil {
		if _, ok := g.Surface.(surfacegeom.Plane); !ok || g.RetainedTag {
			return false, false
		}
		rec, ok := capplane.OfRecord(g.CapRecord)
		if !ok {
			return false, false
		}
		// The face lies on its record's plane up to a displacement along
		// that plane's normal, so a record not parallel to P never meets
		// P in an area.
		if !rec.ParallelTo(plane) {
			return false, true
		}
		gp, ok := capplane.OfFace(g)
		if !ok {
			return false, false
		}
		if !gp.Same(plane) {
			return false, true
		}
		side, ok := outwardSense(plane, g)
		if !ok {
			return false, false
		}
		return side == want, true
	}
	axis, ok := o.axis, o.axisOK
	if !ok {
		axis, ok = brepWallAxis(o.body.Payload, g)
	}
	if !ok || !plane.Crosses(axis) {
		return false, false
	}
	return false, true
}

// outwardSense is the side of plane f's outward normal points to: +1 or -1.
// ok is false for a face with no Plane tag or a normal in the plane.
func outwardSense(plane capplane.Plane, f *topo.Face) (int, bool) {
	tag, ok := f.Surface.(surfacegeom.Plane)
	if !ok {
		return 0, false
	}
	side := plane.Sense(tag.Frame.N())
	if f.Reversed {
		side = -side
	}
	return side, side != 0
}

// recordAxis is the exact direction every face with no CapRecord of a prism
// or stacked record is swept along. ok is false for any other payload.
func recordAxis(payload topo.Payload) ([3]*big.Rat, bool) {
	switch p := payload.(type) {
	case prismrecord.Payload:
		return capplane.Axis(p.Frame, p.Xform)
	case stackedrecord.Payload:
		return capplane.Axis(p.Frame, p.Xform)
	default:
		return [3]*big.Rat{}, false
	}
}

// brepWallAxis is the exact direction a brep wall(k) face is swept along: its
// own record face's axis. ok is false for any other face, for any other
// payload, and for every face of a brep record carrying band patches, which
// are neither caps nor walls.
func brepWallAxis(payload topo.Payload, g *topo.Face) ([3]*big.Rat, bool) {
	p, ok := payload.(breprecord.Payload)
	if !ok || len(p.LoopBands) > 0 || p.BossShell != nil || p.PocketShell != nil || len(g.Origins) == 0 {
		return [3]*big.Rat{}, false
	}
	var k int
	if _, err := fmt.Sscanf(g.Origins[0].Role, "wall(%d)", &k); err != nil ||
		g.Origins[0].Role != fmt.Sprintf("wall(%d)", k) || k < 0 || k >= len(p.Faces) || p.Faces[k].Planar() {
		return [3]*big.Rat{}, false
	}
	return capplane.Axis(p.Faces[k].Frame, p.Xform)
}

func facesOf(b *topo.Body) []*topo.Face {
	var out []*topo.Face
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			out = append(out, s.Faces...)
		}
	}
	return out
}
