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
      icon: useBaseUrl('img/ic-theme.svg'),
      type: 'theme',
      label: `I want a new WordPress <span class="${component}__label-highlight">theme</span>`,
      link: useBaseUrl('/docs/theme'),
    },
    {
      bg: useBaseUrl('img/ic-plugin-bg.svg'),
      icon: useBaseUrl('http://gazebosim.org/assets/logos/gazebo_vert_pos-faad8cc37ab336f850e549077ef5831e5098034532113b06328dfd70355fb8f7.svg'),
      type: 'plugin',
      label: `在 <span class="${component}__label-highlight">Gazebo</span> 中尝试麦克纳姆轮以及舵轮 `,
      link: useBaseUrl('/docs/plugin'),
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
          <div className={`${component}__label`} dangerouslySetInnerHTML={{__html: label}}></div>
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
          title={'Choose one of the options to get started'}
          subtitle={'We\'ve built a bot to automate the tedious process of setting up a new project, so you can focus on your code. Just choose where you want to start.'}
        />
        <div className={`${component}__content`}>
          {items}
        </div>
      </Container>
    </div>
  );
};
