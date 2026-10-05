package export

import (
	"archive/zip"
	"bytes"
	"context"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

const threeMFContentTypes = `<?xml version="1.0" encoding="UTF-8"?>` + "\n" +
	`<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">` +
	`<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>` +
	`<Override PartName="/3D/3dmodel.model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>` +
	`</Types>`

const threeMFRelationships = `<?xml version="1.0" encoding="UTF-8"?>` + "\n" +
	`<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">` +
	`<Relationship Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" ` +
	`Target="/3D/3dmodel.model"/></Relationships>`

// ThreeMF writes one body as a deterministic 3MF Core package at the requested
// chord tolerance. Solids become model objects; sheets become surface objects.
// The mesh defaults to decad.VerifyNone. Pass decad.WithVerification to request
// stronger proofs. ctx, w, and body must not be nil.
func ThreeMF(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: 3MF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: 3MF: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}

	var archive bytes.Buffer
	zw := zip.NewWriter(&archive)
	if err := writeThreeMFPart(zw, "[Content_Types].xml", threeMFContentTypes); err != nil {
		return err
	}
	if err := writeThreeMFPart(zw, "_rels/.rels", threeMFRelationships); err != nil {
		return err
	}
	model, err := zw.Create("3D/3dmodel.model")
	if err != nil {
		return fmt.Errorf("export: 3MF model part: %w", err)
	}
	if err := writeThreeMFModel(ctx, model, body.Kind(), mesh); err != nil {
		return err
	}
	if err := zw.Close(); err != nil {
		return fmt.Errorf("export: 3MF archive: %w", err)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	data := archive.Bytes()
	for len(data) > 0 {
		if err := ctx.Err(); err != nil {
			return err
		}
		chunk := data
		if len(chunk) > 32*1024 {
			chunk = chunk[:32*1024]
		}
		n, err := w.Write(chunk)
		if ctxErr := ctx.Err(); ctxErr != nil {
			return ctxErr
		}
		if err != nil {
			return fmt.Errorf("export: 3MF write failed: %w", err)
		}
		if n != len(chunk) {
			return fmt.Errorf("export: 3MF write failed: %w", io.ErrShortWrite)
		}
		data = data[n:]
	}
	return ctx.Err()
}

func writeThreeMFPart(zw *zip.Writer, name, content string) error {
	w, err := zw.Create(name)
	if err != nil {
		return fmt.Errorf("export: 3MF part %s: %w", name, err)
	}
	if _, err := io.WriteString(w, content); err != nil {
		return fmt.Errorf("export: 3MF part %s: %w", name, err)
	}
	return nil
}

func writeThreeMFModel(ctx context.Context, w io.Writer, kind decad.BodyKind, mesh *decad.Mesh) error {
	objectType := "model"
	if kind == decad.BodySheet {
		objectType = "surface"
	}
	sw := stickyWriter{w: w}
	sw.printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
	sw.printf("<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n")
	sw.printf("<resources><object id=\"1\" type=\"%s\"><mesh><vertices>\n", objectType)
	for _, v := range mesh.Vertices() {
		if err := ctx.Err(); err != nil {
			return err
		}
		sw.printf("<vertex x=\"%s\" y=\"%s\" z=\"%s\"/>\n", fmtCoord(v.X), fmtCoord(v.Y), fmtCoord(v.Z))
		if sw.err != nil {
			return fmt.Errorf("export: 3MF model part: %w", sw.err)
		}
	}
	sw.printf("</vertices><triangles>\n")
	for _, tri := range mesh.Triangles() {
		if err := ctx.Err(); err != nil {
			return err
		}
		sw.printf("<triangle v1=\"%d\" v2=\"%d\" v3=\"%d\"/>\n", tri[0], tri[1], tri[2])
		if sw.err != nil {
			return fmt.Errorf("export: 3MF model part: %w", sw.err)
		}
	}
	sw.printf("</triangles></mesh></object></resources><build><item objectid=\"1\"/></build></model>\n")
	if sw.err != nil {
		return fmt.Errorf("export: 3MF model part: %w", sw.err)
	}
	return nil
}
