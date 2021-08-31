import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Heading from './../../components/heading/heading';
import Container from './../../components/container/container';
import Arrow from './../../components/arrow/arrow';

export default function GetStarted() {

  const component = 'shift-get-started';

  const itemsData = [
    {
      bg: useBaseUrl('img/ic-theme-bg.svg'),
      icon: useBaseUrl('img/homepage/3508.png'),
      type: 'theme',
      label: `在仿真与实物中使用 <span class="${component}__label-highlight">3508</span>`,
      link: useBaseUrl('quick_start/rm-controls_101'),
    },
    {
      bg: useBaseUrl('img/ic-plugin-bg.svg'),
      icon: useBaseUrl('http://gazebosim.org/assets/logos/gazebo_vert_pos-faad8cc37ab336f850e549077ef5831e5098034532113b06328dfd70355fb8f7.svg'),
      type: 'plugin',
      label: `在 <span class="${component}__label-highlight">Gazebo</span> 中操控麦克纳姆轮以及舵轮底盘 `,
      link: useBaseUrl('quick_start/gazebo_chassis'),
    },
  ];

  const items = itemsData.map((item, index) => {
    const {
      bg,
      icon,
      type,
      label,
      link,
    } = item;

    return (
      <div className={`${component}__item ${component}__item--${type}`} key={index}>
        <a className={`${component}__link`} href={link}>
          <div className={`${component}__icon`}>
            <img className={`${component}__icon-bg`} src={bg} />
            <img className={`${component}__icon-img`} src={icon} />
          </div>
          <div className={`${component}__label`} dangerouslySetInnerHTML={{__html: label}}/>
          <Arrow componentClass={component} />
        </a>
      </div>
    )
  });

  return (
    <div className={component}>
      <Container
        componentClass={component}
        size={'small'}
      >
        <Heading
          componentClass={component}
          title={'选择一种入门的方式'}
          subtitle={'你可以通过手上常用的 RM3508 电机学习如何在仿真和现实中使用 rm-controls；如果你觉得单个电机过于简单，可以尝试在仿真中操控麦克纳姆轮和舵轮底盘'}
        />
        <div className={`${component}__content`}>
          {items}
        </div>
      </Container>
    </div>
  );
};
