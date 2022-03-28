const lightCodeTheme = require("prism-react-renderer/themes/github");
const darkCodeTheme = require("prism-react-renderer/themes/dracula");
const math = require("remark-math");
const katex = require("rehype-katex");

/** @type {import('@docusaurus/types').DocusaurusConfig} */
module.exports = {
  title: "rm-controls documentation",
  tagline: "",
  url: "https://rm-controls.github.io",
  baseUrl: "/",
  onBrokenLinks: "warn",
  onBrokenMarkdownLinks: "warn",
  favicon: "img/favicon.ico",
  organizationName: "rm-controls",
  projectName: "rm-controls.github.io",
  trailingSlash: false,
  i18n: {
    defaultLocale: "zh-CN",
    locales: ["en", "zh-CN"],
    localeConfigs: {
      en: {
        label: "English",
        direction: "ltr",
      },
      "zh-CN": {
        label: "中文",
        direction: "ltr",
      },
    },
  },
  themes: [
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        hashed: true,
        language: ["en", "zh"],
        docsRouteBasePath: "/",
        docsDir: "current_docs",
        indexBlog: false,
      },
    ],
  ],
  themeConfig: {
    announcementBar: {
      id: "star",
      content:
        '⭐️ If you like rm-controls, give it a star on <a target="_blank" rel="noopener noreferrer" href="https://github.com/rm-controls/rm_control">rm_control</a> and <a target="_blank" rel="noopener noreferrer" href="https://github.com/rm-controls/rm_controllers">rm_controllers</a> ! ⭐️',
    },
    navbar: {
      title: "rm-controls",
      logo: {
        alt: "My Site Logo",
        src: "img/logo.png",
      },
      items: [
        {
          href: "/get-started",
          position: "right",
          label: "Get Started",
        },
        {
          type: "doc",
          docId: "overview/intro",
          position: "right",
          label: "Docs",
        },
        {
          type: "localeDropdown",
          position: "right",
        },
        {
          href: "https://github.com/rm-controls",
          position: "right",
          className: "header-github-link",
          "aria-label": "GitHub repository",
        },
      ],
      hideOnScroll: true,
    },
    footer: {
      style: "dark",
      copyright: `Copyright © ${new Date().getFullYear()}. Distributed by BSD 3-Clause License`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  },
  presets: [
    [
      "@docusaurus/preset-classic",
      {
        docs: {
          path: "current_docs",
          remarkPlugins: [math],
          rehypePlugins: [katex],
          showLastUpdateAuthor: false,
          showLastUpdateTime: true,
          routeBasePath: "/",
          sidebarPath: require.resolve("./sidebars.js"),
          editUrl:
            "https://github.com/rm-controls/rm-controls.github.io/tree/master",
        },
        blog: {
          showReadingTime: true,
          editUrl:
            "https://github.com/rm-controls/rm-controls.github.io/tree/master",
        },
        theme: {
          customCss: require.resolve("./src/scss/application.scss"),
        },
      },
    ],
  ],
  stylesheets: [
    {
      href: "https://cdn.jsdelivr.net/npm/katex@0.13.11/dist/katex.min.css",
      integrity:
        "sha384-Um5gpz1odJg5Z4HAmzPtgZKdTBHZdw8S29IecapCSB31ligYPhHQZMIlWLYQGVoc",
      crossorigin: "anonymous",
    },
  ],
  plugins: ["docusaurus-plugin-sass"],
};
