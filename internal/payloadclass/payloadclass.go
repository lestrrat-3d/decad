// Package payloadclass lets the public test kit (decadtest) read which payload
// class built a body. The root package installs readers when it loads. The
// body wire encoder also asks whether the body holds a faceted payload.
package payloadclass

import "fmt"

// Reader returns the payload type name of a *decad.Body, as
// TypeName prints it, or "" for any other value. The root
// package sets it once at load time.
var Reader func(body any) string

// FacetedReader reports whether a root body holds its own faceted payload.
var FacetedReader func(body any) bool

// HoldsFacets reports false until root installs its reader.
func HoldsFacets(body any) bool {
	return FacetedReader != nil && FacetedReader(body)
}

// Of returns the payload class name of body, or "" before the root package has
// installed Reader or when body is not a body handle.
func Of(body any) string {
	if Reader == nil {
		return ""
	}
	return Reader(body)
}

// TypeName names a payload's type in a public error message. A payload
// that states a SurveyClass prints as the root type it replaced, "decad."
// followed by that class, so moving a payload between packages leaves the
// message unchanged. Any other payload prints its Go type name.
func TypeName(payload any) string {
	if named, ok := payload.(interface{ SurveyClass() string }); ok {
		return "decad." + named.SurveyClass()
	}
	return fmt.Sprintf("%T", payload)
}
