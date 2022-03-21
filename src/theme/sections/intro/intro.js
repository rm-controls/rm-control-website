import React from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import useBaseUrl from "@docusaurus/useBaseUrl";
import Container from "./../../components/container/container";
import Button from "./../../components/button/button";
import Translate from "@docusaurus/Translate";

export default function Intro() {
  const context = useDocusaurusContext();
  const { siteConfig = {} } = context;

  const component = "shift-intro";

  return (
    <div className={component}>
      <Container componentClass={component} size={"medium"}>
        <div className={`${component}__title`}>
          rm-controls
          <br />
          <Translate id="homepage.intro1">{"电控方案"}</Translate>
        </div>
        <div className={`${component}__image`}>
          <img
            className={`${component}__image-bg`}
            src={useBaseUrl("img/ic-intro.svg")}
          />
          <img
            className={`${component}__image-img`}
            src={useBaseUrl("img/homepage/swerve_robot.png")}
          />
        </div>
        <div className={`${component}__content`}>
          {siteConfig.tagline}
          <Translate id="homepage.intro">
            {
              "一套在 PC 上运行的无下位机、视控一体软件和配套硬件，基于 ros-controls 的硬件和仿真接口以及配套的控制器，用于开发 RoboMaster 机器人和高性能机器人。"
            }
          </Translate>
        </div>
        <Button
          componentClass={component}
          label={"Get Started"}
          href={useBaseUrl("/get-started")}
        />
      </Container>
    </div>
  );
}
