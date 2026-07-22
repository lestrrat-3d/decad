package decad_test

import (
	"encoding/json"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// countRolePrefix counts the body's faces carrying at least one origin role
// whose name starts with prefix — e.g. "fillet(" or "chamfer(".
func countRolePrefix(b *decad.Body, prefix string) int {
	n := 0
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, prefix) {
				n++
				break
			}
		}
	}
	return n
}

func TestDuplicate(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	inst, err := body.Duplicate()
	require.NoError(t, err)
	require.NotSame(t, body, inst, `a duplicate is a fresh body identity`)

	// The inst is identical, independent geometry: same volume, same centroid.
	cVol, err := inst.Volume()
	require.NoError(t, err)
	require.True(t, cVol.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	cCent, err := inst.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 50.0, cCent.Value.X, 1e-9)
	require.InDelta(t, 30.0, cCent.Value.Y, 1e-9)
	require.InDelta(t, 5.0, cCent.Value.Z, 1e-9)
	requireManifold(t, inst)

	// The source stays LIVE — both bodies are in the model, source first —
	// and remains fully operable (a duplicate consumes nothing).
	require.Equal(t, []*decad.Body{body, inst}, doc.Bodies())
	srcVol, err := body.Volume()
	require.NoError(t, err)
	require.True(t, srcVol.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	again, err := body.Duplicate()
	require.NoError(t, err, `the source takes further ops — it was never retired`)
	require.NotNil(t, again)

	// The inst's own step is its origin; the inst step depends on the source.
	require.Equal(t, decad.StepRef(1), inst.Origin().Step)
	recipe := doc.Recipe()
	require.Equal(t, decad.OpDuplicate, recipe.Steps[1].Op)
	require.Equal(t, []decad.StepRef{0}, recipe.Steps[1].Inputs)
	// A duplicate records no motion: its Placement is the zero record (absent).
	require.Equal(t, decad.TransformRecord{}, recipe.Steps[1].Placement)

	// The recipe round-trips through JSON with the new op.
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got)
}

func TestPlacedCopy(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	shift, err := r3.Translation(r3.NewVec(200, 0, 0))
	require.NoError(t, err)

	inst, err := body.PlacedCopy(shift)
	require.NoError(t, err)

	// The motion is rigid: volume is untouched and the centroid MOVES by t.
	vol, err := inst.Volume()
	require.NoError(t, err)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	c, err := inst.Centroid()
	require.NoError(t, err)
	want := shift.Apply(r3.NewVec(50, 30, 5))
	require.InDelta(t, want.X, c.Value.X, 1e-9)
	require.InDelta(t, want.Y, c.Value.Y, 1e-9)
	require.InDelta(t, want.Z, c.Value.Z, 1e-9)
	requireManifold(t, inst)

	// The source stays live and unmoved: an instance is placed from a part
	// that itself does not move.
	require.Equal(t, []*decad.Body{body, inst}, doc.Bodies())
	sc, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 50.0, sc.Value.X, 1e-9, `the source did not move`)

	// A second placed instance reuses the same source — the bolt-pattern case.
	shift2, err := r3.Translation(r3.NewVec(-200, 0, 0))
	require.NoError(t, err)
	copy2, err := body.PlacedCopy(shift2)
	require.NoError(t, err)
	c2, err := copy2.Centroid()
	require.NoError(t, err)
	require.InDelta(t, -150.0, c2.Value.X, 1e-9)

	// The placed-inst step records its motion in Placement and depends on the
	// source; the recipe round-trips through JSON.
	recipe := doc.Recipe()
	require.Equal(t, decad.OpPlacedCopy, recipe.Steps[1].Op)
	require.Equal(t, []decad.StepRef{0}, recipe.Steps[1].Inputs)
	require.NotEqual(t, decad.TransformRecord{}, recipe.Steps[1].Placement, `a placed inst records its motion`)
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got)
}

func TestPlacedCopyZeroTransformRejected(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	_, err = body.PlacedCopy(r3.Transform{})
	require.ErrorIs(t, err, decad.ErrDegenerate, `the zero transform names no placement`)

	// The identity is a valid no-op motion — the same geometry at a new identity.
	inst, err := body.PlacedCopy(r3.Identity())
	require.NoError(t, err)
	c, err := inst.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 50.0, c.Value.X, 1e-9)

	// The rejection left the source live and the recipe untouched save the
	// one valid identity inst.
	require.Equal(t, []*decad.Body{body, inst}, doc.Bodies())
}

func TestDuplicatePreservesFilletRoles(t *testing.T) {
	// A filleted prism's blend-role provenance is part of its re-evaluable
	// record, so a Duplicate re-mints its own fillet(i,j) roles: the copy has
	// the SAME count of fillet-role faces as the source (modify §9).
	_, box := filletBox(t)
	src, err := box.Fillet(verticalEdges(), units.Millimeters(10))
	require.NoError(t, err)
	srcFillets := countRolePrefix(src, "fillet(")
	require.Equal(t, 4, srcFillets, `the source has one fillet role per rounded corner`)

	inst, err := src.Duplicate()
	require.NoError(t, err)
	require.Equal(t, srcFillets, countRolePrefix(inst, "fillet("),
		`the duplicate re-mints the same fillet roles from its own record`)

	// The re-minted roles index the COPY's own step, never the source's — a
	// prism-derived body's roles are built from its own record (modify §9).
	for _, f := range inst.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "fillet(") || strings.HasPrefix(o.Role, "side(") {
				require.Equal(t, inst.Origin().Step, o.Step, `a copy's roles index its own record`)
			}
		}
	}

	// FaceCreatedBy over one of the copy's own fillet refs still selects a
	// blend wall — provenance is queryable on the duplicate.
	var filletRef *decad.FeatureRef
	for _, f := range inst.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "fillet(") {
				ref := o
				filletRef = &ref
			}
		}
	}
	require.NotNil(t, filletRef, `the duplicate carries a fillet role`)
	faces, err := decad.Faces(decad.FaceCreatedBy(*filletRef)).SelectFaces(inst)
	require.NoError(t, err)
	require.NotEmpty(t, faces, `a fillet role selects a face on the duplicate`)
}

func TestPlacedCopyPreservesFilletRoles(t *testing.T) {
	// The same guarantee under a non-identity placement: PlacedCopy re-evaluates
	// the payload under the composed motion and re-mints the fillet roles.
	_, box := filletBox(t)
	src, err := box.Fillet(verticalEdges(), units.Millimeters(10))
	require.NoError(t, err)
	srcFillets := countRolePrefix(src, "fillet(")
	require.Equal(t, 4, srcFillets)

	shift, err := r3.Translation(r3.NewVec(300, 0, 0))
	require.NoError(t, err)
	inst, err := src.PlacedCopy(shift)
	require.NoError(t, err)
	require.Equal(t, srcFillets, countRolePrefix(inst, "fillet("),
		`the placed copy re-mints the same fillet roles`)
	requireManifold(t, inst)
}

func TestPlacedCopyPreservesChamferRoles(t *testing.T) {
	// Chamfer shares the blend-descriptor machinery, so a copy re-mints its
	// chamfer(i,j) roles the same way a fillet's are re-minted.
	_, box := filletBox(t)
	src, err := box.Chamfer(verticalEdges(), units.Millimeters(8))
	require.NoError(t, err)
	srcChamfers := countRolePrefix(src, "chamfer(")
	require.Equal(t, 4, srcChamfers, `the source has one chamfer role per bevelled corner`)

	shift, err := r3.Translation(r3.NewVec(0, 300, 0))
	require.NoError(t, err)
	inst, err := src.PlacedCopy(shift)
	require.NoError(t, err)
	require.Equal(t, srcChamfers, countRolePrefix(inst, "chamfer("),
		`the placed copy re-mints the same chamfer roles`)

	dup, err := src.Duplicate()
	require.NoError(t, err)
	require.Equal(t, srcChamfers, countRolePrefix(dup, "chamfer("),
		`the duplicate re-mints the same chamfer roles`)
}

func TestDuplicatePlainExtrudeHasNoBlendRoles(t *testing.T) {
	// A plain extrude carries no blend descriptors, so its copy gains no spurious
	// fillet/chamfer roles — the empty descriptor path is a no-op.
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Zero(t, countRolePrefix(body, "fillet("))
	require.Zero(t, countRolePrefix(body, "chamfer("))

	inst, err := body.Duplicate()
	require.NoError(t, err)
	require.Zero(t, countRolePrefix(inst, "fillet("), `a plain extrude copy has no fillet roles`)
	require.Zero(t, countRolePrefix(inst, "chamfer("), `a plain extrude copy has no chamfer roles`)
}

func TestDuplicatePreservesFacetedProvenance(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)

	cut, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	// The cut result's faces carry each source's upstream FeatureRef origins.
	want := map[decad.FeatureRef]struct{}{}
	for _, f := range cut.Faces() {
		for _, o := range f.Origins() {
			want[o] = struct{}{}
		}
	}
	require.NotEmpty(t, want)

	inst, err := cut.Duplicate()
	require.NoError(t, err)
	// The inst's OWN step is its Body.Origin(), keyed to the inst step.
	require.Equal(t, decad.FeatureRef{Step: inst.Origin().Step, Role: "body"}, inst.Origin())

	// The inst carries EXACTLY the same upstream origins — verbatim, not
	// re-keyed to the inst's own step: a faceted inst preserves provenance.
	got := map[decad.FeatureRef]struct{}{}
	for _, f := range inst.Faces() {
		for _, o := range f.Origins() {
			got[o] = struct{}{}
		}
	}
	require.Equal(t, want, got)

	// FaceCreatedBy still finds a copied face by an upstream ref the source had.
	for ref := range want {
		faces, err := decad.Faces(decad.FaceCreatedBy(ref)).SelectFaces(inst)
		require.NoError(t, err)
		require.NotEmpty(t, faces, `an upstream ref still selects a face on the inst`)
	}

	// The source stayed live: the cut body is still operable after the inst.
	require.Contains(t, doc.Bodies(), cut)
}
