import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Container from './../../components/container/container';
import Button from './../../components/button/button';

export default function Intro() {
  const context = useDocusaurusContext();
  const {siteConfig = {}} = context;

  const component = 'shift-intro';

  return (
    <div className={component}>
      <Container
        componentClass={component}
        size={'medium'}
      >
        <div className={`${component}__title`} dangerouslySetInnerHTML={{__html: 'rm-controls<br/>电控方案'}}/>
        <div className={`${component}__image`}>
          <img className={`${component}__image-bg`} src={useBaseUrl('img/ic-intro.svg')} />
          <img className={`${component}__image-img`} src={useBaseUrl('img/homepage/swerve_robot.png')} />
        </div>
        <div className={`${component}__content`}>
          {siteConfig.tagline}
          一套在 PC 上运行的无下位机、视控一体软件和配套硬件，基于 ros-controls 的硬件和仿真接口以及配套的控制器，用于开发 RoboMaster 机器人和高性能机器人。
        </div>
        <Button 
          componentClass={component}
          label={'Get Started'}
          href={useBaseUrl('/get-started')}
        />
      </Container>
    </div>
  );
}
