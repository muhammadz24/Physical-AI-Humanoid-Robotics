// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const {themes} = require('prism-react-renderer');
const lightTheme = themes.github;
const darkTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn Physical AI, ROS 2, and Humanoid Robotics from Fundamentals to Advanced',
  favicon: 'img/logo.svg',

  // Set the production url of your site here (Vercel deployment)
  url: 'https://physical-ai-humanoid-robotics-mz24.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // CRITICAL: Must be root '/' for Vercel clean URLs
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'muhammadz24',
  projectName: 'Physical-AI-Humanoid-Robotics',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/', // Serve docs at root path for immediate sidebar visibility
          // Breadcrumb navigation (FR-010)
          breadcrumbs: true,
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Social card
      image: 'img/social-card.jpg',

      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        hideOnScroll: true,
        logo: {
          alt: 'Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            to: '/author',
            label: 'Book Author',
            position: 'right',
            className: 'navbar-author-link',
          },
        ],
      },

      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introduction',
                to: '/',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org',
              },
              {
                label: 'Gazebo Documentation',
                href: 'https://gazebosim.org/docs',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Content: CC BY-SA 4.0, Code: MIT.`,
      },

      // Prism syntax highlighting configuration (FR-003, T050)
      prism: {
        theme: lightTheme,
        darkTheme: darkTheme,
        additionalLanguages: ['python', 'yaml', 'bash'],
      },

      // Dark mode configuration (FR-009, US5)
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },

      // Algolia search (will be configured later for FR-011)
      // For now, using default search
    }),

  // Code block features (FR-003): line numbers, copy button
};

module.exports = config;
