const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').DocusaurusConfig} */
module.exports = {
  title: 'rm-controls documentation',
  tagline: '',
  url: 'https://rm-controls.github.io',
  baseUrl: '/rm-controls-docs/',
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'rm-controls',
  projectName: 'rm-controls-docs',
  trailingSlash: false,
  i18n: {
    defaultLocale: 'zh-cn',
    locales: ['en', 'zh-cn'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      'zh-cn': {
        label: '中文',
        direction: 'ltr',
      },
    },
  },
  themeConfig: {
    navbar: {
      title: 'rm-controls',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'doc',
          docId: 'overview/intro',
          position: 'right',
          label: 'Docs',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/rm-controls',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
      hideOnScroll: true,
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()}. Distributed by BSD 3-Clause License`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          path: 'current_docs',
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          editUrl: 'https://github.com/rm-controls/rm-controls-docs',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          editUrl:
            'https://github.com/rm-controls/rm-controls-docs',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
  plugins: [
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        hashed: true,
        language: ["en", "zh"],
        docsRouteBasePath: "/docs/",
      },
    ],
  ],
};
