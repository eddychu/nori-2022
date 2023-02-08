#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
NORI_NAMESPACE_BEGIN

    class AOIntegrator : public Integrator {
    public:
        AOIntegrator(const PropertyList &props) {
        }

        /// Compute the radiance value for a given ray. Just return green here
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);
            auto sample = sampler->next2D();
            auto sampleDir = Warp::squareToCosineHemisphere(sample);

            auto sampleDirWorld = its.shFrame.toWorld(sampleDir).normalized();

            Ray3f shadowRay(its.p, sampleDirWorld, Epsilon, std::numeric_limits<float>::infinity());
            Intersection shadowIts;
            if (scene->rayIntersect(shadowRay, shadowIts))
                return Color3f(0.0f);
            return Color3f(1.0f);
        }

        /// Return a human-readable description for debugging purposes
        std::string toString() const {
            return "AOIntegrator[]";
        }

    };

    NORI_REGISTER_CLASS(AOIntegrator, "ao");
NORI_NAMESPACE_END