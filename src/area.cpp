#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>
NORI_NAMESPACE_BEGIN

    class AreaLight: public Emitter {
        public:
            AreaLight(const PropertyList &props) {
                m_radiance = props.getColor("radiance", Color3f(1.0f));
            }
            std::string toString() const {
                return "AreaLight[]";
            }

            virtual Color3f le(const Point3f& p, const Point3f& n, const Point3f& target) const override {
				return n.dot(target - p) > 0 ? m_radiance : Color3f(0.0f);
            }

            virtual Color3f le(const Ray3f& ray) const override {
                return m_radiance;
            }

          
        private:
                Color3f m_radiance;
    };



    NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END