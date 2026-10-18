package capblendsurvey

import (
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/r3"
)

// This file reads cap-blend face tags and placed frames for the
// exact normal model in internal/capband/normal_model.go.

type capPatchModel = capband.NormalModel

// capPatchNormalModel adapts the published face and placed frame for capband.
func capPatchNormalModel(f *topo.Face, pl prismPayload, g capPatchGeom, p r3.Vec) (capPatchModel, bool) {
	sinH, cosH, origin, axis, ok := capblend.ConeTagTerms(f)
	if !ok {
		return capPatchModel{}, false
	}
	world, ok := survey2d.NewPlacedFrameMap(pl.Frame, pl.Xform)
	if !ok {
		return capPatchModel{}, false
	}
	return capband.PatchNormalModel(capband.NormalInput{
		SinH: sinH, CosH: cosH, Origin: origin, Axis: axis, Pull: p,
		World: world, CU: g.CU, CV: g.CV, CapZ: g.CapZ,
		Radius: g.CapRadius, Th0: g.Th0, Reversed: f.Reversed,
	})
}
