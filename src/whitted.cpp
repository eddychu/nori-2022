#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <algorithm>
NORI_NAMESPACE_BEGIN

    class WhittedIntegrator : public Integrator {
    public:
        WhittedIntegrator(const PropertyList &props) {
        }

        /// Compute the radiance value for a given ray. Just return green here
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);
            Color3f color = Color3f(0.0f);
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord record;
                record.position = its.p;
                record.target = ray.o;
                record.normal = its.shFrame.n;
                color += its.mesh->getEmitter()->le(ray);
            }
            std::vector<Mesh*> emitters;
            std::copy_if(scene->getMeshes().begin(), scene->getMeshes().end(), std::back_inserter(emitters), [](const Mesh* mesh) {
                return mesh->isEmitter();
            });
			if (emitters.size() == 0)
				return color;
			int emitterIndex = (int)sampler->next1D() * emitters.size();
			Mesh* emitter = emitters[emitterIndex];
            auto lightSample = emitter->sample(sampler);
            Vector3f shadowRayDir = lightSample.p - its.p;
            float shadowRayDist = shadowRayDir.norm();
            shadowRayDir.normalize();
            Ray3f shadowRay(its.p, shadowRayDir, Epsilon, shadowRayDist - Epsilon);
            if (!scene->rayIntersect(shadowRay)) {
                float cosTheta = std::abs(its.shFrame.cosTheta(its.shFrame.toLocal(shadowRayDir)));
                float cosPhi = std::abs(lightSample.n.dot(-shadowRayDir));
                float g = cosTheta * cosPhi / shadowRayDist / shadowRayDist;
                BSDFQueryRecord bRec(its.toLocal(shadowRayDir), its.toLocal(-ray.d), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bRec);
                color += emitter->getEmitter()->le(lightSample.p, lightSample.n, its.p) * f * g * emitters.size() / lightSample.pdf;
            }
            return color;
        }

        /// Return a human-readable description for debugging purposes
        std::string toString() const {
            return "WhittedIntegrator[]";
        }

    };

    NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END